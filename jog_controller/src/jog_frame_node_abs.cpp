#include <cfloat>
#include <jog_controller/jog_frame_node_abs.h>
#include <thread>
#include <boost/thread/thread.hpp>
#include <math.h>
#include <typeinfo>

namespace jog_frame {

JogFrameNodeAbs::JogFrameNodeAbs()
{
  ros::NodeHandle gnh, pnh("~");

  ROS_WARN("Absolute mode enabled!");
  initInteractiveMarkers();

  //pnh.param<std::string>("target_link", target_link_, "link_6");
  //pnh.param<std::string>("group", group_name_);
  pnh.param<double>("time_from_start", time_from_start_, 0.5);
  pnh.param<bool>("use_action", use_action_, false);
  pnh.param<bool>("intermittent", intermittent_, false);

  std::vector<std::string> group_names;
  gnh.getParam("/jog_frame_node/group_names", group_names);
  group_name_ = group_names.size() > 0 ? group_names[0] : "";

  std::vector<std::string> link_names;
  gnh.getParam("/jog_frame_node/link_names", link_names);
  target_link_ = link_names.size() > 0 ? link_names[0] : "";

  jog_frame_abs_pub_ = gnh.advertise<jog_msgs::JogFrame>( "jog_frame", 1);
  avoid_collisions_ = true;

  if (not use_action_ && intermittent_)
  {
    ROS_WARN("'intermittent' param should be true with 'use_action'. Assuming to use action'");
    use_action_ = true;
  }
  // Exclude joint list
  gnh.getParam("exclude_joint_names", exclude_joints_);

  if (get_controller_list() < 0)
  {
    ROS_ERROR("get_controller_list failed. Aborted.");
    ros::shutdown();
    return;
  }
  ROS_INFO_STREAM("controller_list:");
  for (auto it=cinfo_map_.begin(); it!=cinfo_map_.end(); it++)
  {
    auto cinfo = it->second;
    ROS_INFO_STREAM("- " << it->first);
    for (int i=0; i<cinfo.joints.size(); i++)
    {
      ROS_INFO_STREAM("  - " << cinfo.joints[i]);
    }    
  }  

  // Create subscribers
  joint_state_sub_ = gnh.subscribe("joint_states", 10, &JogFrameNodeAbs::joint_state_cb, this);
  jog_frame_sub_ = gnh.subscribe("jog_frame", 10, &JogFrameNodeAbs::jog_frame_cb, this);
  fk_client_ = gnh.serviceClient<moveit_msgs::GetPositionFK>("compute_fk");  
  ik_client_ = gnh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");  
  ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
  
  if (use_action_)
  {
    // Create action client for each controller
    for (auto it=cinfo_map_.begin(); it!=cinfo_map_.end(); it++)
    {
      auto controller_name = it->first;
      auto controller_info = it->second;
      auto action_name = controller_name + "/" + controller_info.action_ns;
      
      traj_clients_[controller_name] = new TrajClient(action_name, true);
    }
    for (auto it=cinfo_map_.begin(); it!=cinfo_map_.end(); it++)
    {
      auto controller_name = it->first;
      auto controller_info = it->second;
      auto action_name = controller_name + "/" + controller_info.action_ns;
      
      for(;;)
      {
        if (traj_clients_[controller_name]->waitForServer(ros::Duration(1)))
        {
          ROS_INFO_STREAM(action_name << " is ready.");
          break;
        }
        ROS_WARN_STREAM("Waiting for " << action_name << " server...");
      }
    }
  }
  else
  {
    // Create publisher for each controller
    for (auto it=cinfo_map_.begin(); it!=cinfo_map_.end(); it++)
    {
      auto controller_name = it->first;
      traj_pubs_[controller_name] = gnh.advertise<trajectory_msgs::JointTrajectory>(controller_name + "/command", 10);
    }
  }

  // Set the marker to the end effectors pose
  resetInteractiveMarker();

  // Create seperate thread for continous jogging towards the target position.
  boost::thread worker_thread(&JogFrameNodeAbs::follow_absolute_pose_thread, this); 
}

/**
 * @brief Callback function for the topic jog_frame
 * 
*/
void JogFrameNodeAbs::jog_frame_cb(jog_msgs::JogFrameConstPtr msg)
{
  joint_state_.header.stamp = ros::Time::now();

  // In intermittent mode, confirm to all of the action is completed
  if (intermittent_)
  {
    for (auto it=cinfo_map_.begin(); it!=cinfo_map_.end(); it++)
    {
      auto controller_name = it->first;
      actionlib::SimpleClientGoalState state = traj_clients_[controller_name]->getState();
      if (state == actionlib::SimpleClientGoalState::ACTIVE)
      {
        return;
      }
    }    
  }
  // Update reference frame only if the stamp is older than last_stamp_ + time_from_start_
  if (msg->header.stamp > last_stamp_ ) //+ ros::Duration(time_from_start_))
  {
    // update our reference message for the secondary thread
    frame_id_ = msg->header.frame_id;
    group_name_ = msg->group_name;
    target_link_ = msg->link_name;
    avoid_collisions_ = msg->avoid_collisions;
    //ROS_WARN("req mutex 1");
    std::lock_guard<std::mutex> guard_ref_msg(ref_msg_mutex_);
    //ROS_INFO("mutex 1");
    ref_msg_ = msg;
  }  
  // Update timestamp of the last jog command
  last_stamp_ = msg->header.stamp;
}


/**
 * Worker thread for calulation
 */
void JogFrameNodeAbs::follow_absolute_pose_thread() {
  
  // TODO parametrize
  double rate = 10;
  ros::Rate jogging_rate(rate);

  while(ros::ok()) {
    if(ref_msg_ != nullptr) {
      if(ref_msg_->header.stamp.sec > 0 && ref_msg_->header.stamp.nsec > 0) {
        jogStep(rate);
      }
    }
    
    if(marker_msg_.header.stamp.sec > 0 && marker_msg_.header.stamp.nsec > 0) {
      //ROS_WARN("req mutex 3");
      std::lock_guard<std::mutex> guard_marker_msg(marker_msg_mutex_);
      //ROS_INFO("mutex 3");
      // ROS_INFO("Publish marker");
      marker_msg_.header.stamp = ros::Time::now();
      jog_frame_abs_pub_.publish(marker_msg_);
    }
    
    jogging_rate.sleep();
  }
}


/**
 * @brief get the current pose of the endeffector. Accessible via pose_stamped_
 */
void JogFrameNodeAbs::getFkPose() 
{
  joint_state_.name.clear();
  joint_state_.position.clear();
  joint_state_.velocity.clear();
  joint_state_.effort.clear();
  
  for (auto it=joint_map_.begin(); it!=joint_map_.end(); it++)
  {
    // Exclude joint in exclude_joints_
    if (std::find(exclude_joints_.begin(),
                  exclude_joints_.end(), it->first) != exclude_joints_.end())
    {
      ROS_INFO_STREAM("joint " << it->first << "is excluded from FK");
      continue;
    }
    // Update reference joint_state
    joint_state_.name.push_back(it->first);
    joint_state_.position.push_back(it->second);
    joint_state_.velocity.push_back(0.0);
    joint_state_.effort.push_back(0.0);
  }

  // Update forward kinematics
  moveit_msgs::GetPositionFK fk;

  fk.request.header.frame_id = frame_id_;
  fk.request.header.stamp = ros::Time::now();
  fk.request.fk_link_names.clear();
  fk.request.fk_link_names.push_back(target_link_);
  fk.request.robot_state.joint_state = joint_state_;

  if (fk_client_.call(fk))
  {
    if(fk.response.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      ROS_INFO_STREAM("fk: " << fk.request);
      ROS_WARN("****FK error %d", fk.response.error_code.val);
      return;
    }
    if (fk.response.pose_stamped.size() != 1)
    {
      for (int i=0; i<fk.response.pose_stamped.size(); i++)
      {
        ROS_ERROR_STREAM("fk[" << i << "]:\n" << fk.response.pose_stamped[0]);
      }
    }
    // ROS_WARN("req mutex 4");
    std::lock_guard<std::mutex> guard_pose_stamped(pose_stamped_mutex_);
    // ROS_INFO("mutex 4");
    pose_stamped_ = fk.response.pose_stamped[0];
  }
  else
  {
    ROS_ERROR("Failed to call service /computte_fk");
    return;
  }
}

void JogFrameNodeAbs::jogStep(double rate) 
{
  // get the current pose, accessiable via pose_stamped_
  getFkPose();
  //ROS_WARN("req mutex 5");
  std::lock_guard<std::mutex> guard_pose_stamped(pose_stamped_mutex_);
  //ROS_INFO("mutex 5");

  // Solve inverse kinematics
  moveit_msgs::GetPositionIK ik;

  ik.request.ik_request.group_name = group_name_;
  ik.request.ik_request.ik_link_name = target_link_;
  ik.request.ik_request.robot_state.joint_state = joint_state_;
  ik.request.ik_request.avoid_collisions = avoid_collisions_;
  
  geometry_msgs::Pose act_pose = pose_stamped_.pose;
  geometry_msgs::PoseStamped ref_pose;
  
  ref_pose.header.frame_id = ref_msg_->header.frame_id;
  ref_pose.header.stamp = ros::Time::now();

  // when the target pose is too far away, a pose with a max difference will be calculated directing
  // the pose towards the target pose (basically linear interpolation).

  double dirX = ref_msg_->linear_abs.x - act_pose.position.x;
  double dirY = ref_msg_->linear_abs.y - act_pose.position.y;
  double dirZ = ref_msg_->linear_abs.z - act_pose.position.z;
  double dist = sqrt(((double)dirX*dirX) + ((double)dirY*dirY) + ((double)dirZ*dirZ));

  // do not jog any further, this is close enough
  if(dist < 0.005) {
    return;
  }

  // check whether downscaling is necessary
  // TODO parametrize
  double threshold = 0.2;
  if(dist > threshold) {
    dirX = (dirX / dist) * threshold; 
    dirY = (dirY / dist) * threshold; 
    dirZ = (dirZ / dist) * threshold;
  }
  double nDirX = (dirX * 0.5); 
  double nDirY = (dirY * 0.5); 
  double nDirZ = (dirZ * 0.5);
  ref_pose.pose.position.x = pose_stamped_.pose.position.x + nDirX;
  ref_pose.pose.position.y = pose_stamped_.pose.position.y + nDirY;
  ref_pose.pose.position.z = pose_stamped_.pose.position.z + nDirZ;


  // Apply orientation jog
  tf::Quaternion q_ref, q_act, q_jog;
  tf::quaternionMsgToTF(act_pose.orientation, q_act);
  double angle = sqrt(ref_msg_->angular_delta.x*ref_msg_->angular_delta.x +
                      ref_msg_->angular_delta.y*ref_msg_->angular_delta.y +
                      ref_msg_->angular_delta.z*ref_msg_->angular_delta.z);
  tf::Vector3 axis(0,0,1);
  if (fabs(angle) < DBL_EPSILON)
  {
    angle = 0.0;
  }
  else
  {
    axis.setX(ref_msg_->angular_delta.x/angle);
    axis.setY(ref_msg_->angular_delta.y/angle);
    axis.setZ(ref_msg_->angular_delta.z/angle);
  }

  q_jog.setRotation(axis, angle);
  q_ref = q_jog*q_act;
  quaternionTFToMsg(q_ref, ref_pose.pose.orientation);

  ik.request.ik_request.pose_stamped = ref_pose;

  if (!ik_client_.call(ik))
  {
    ROS_ERROR("Failed to call service /compute_ik");
    return;
  }
  if (ik.response.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_WARN("****IK error %d", ik.response.error_code.val);
    return;
  } 

  auto state = ik.response.solution.joint_state;
  geometry_msgs::PoseStamped pose_check;
  
  // Make sure the solution is valid in joint space
  double error = 0;
  for (int i=0; i<state.name.size(); i++)
  {
    for (int j=0; j<joint_state_.name.size(); j++)
    {
      if (state.name[i] == joint_state_.name[j])
      {
        double e = fabs(state.position[i] - joint_state_.position[j]);
        if (e > error)
        {
          error = e;
        }
        break;
      }
    }
  }
  if (error > M_PI / 2)
  {
    ROS_ERROR_STREAM("**** Validation check Failed: " << error);
    return;
  } 

  publishPose(state);
}

void JogFrameNodeAbs::publishPose(sensor_msgs::JointState state) 
{
  // Publish trajectory message for each controller
  for (auto it=cinfo_map_.begin(); it!=cinfo_map_.end(); it++)
  {
    auto controller_name = it->first;
    auto joint_names = it->second.joints;

    std::vector<double> positions, velocities, accelerations;

    positions.resize(joint_names.size());
    velocities.resize(joint_names.size());
    accelerations.resize(joint_names.size());

    for (int i=0; i<joint_names.size(); i++)
    {
      size_t index = std::distance(state.name.begin(),
                                  std::find(state.name.begin(),
                                            state.name.end(), joint_names[i]));
      if (index == state.name.size())
      {
        ROS_WARN_STREAM("Cannot find joint " << joint_names[i] << " in IK solution");        
      }
      positions[i] = state.position[index];
      velocities[i] = 0;
      accelerations[i] = 0;
    }
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = positions;
    point.velocities = velocities;
    point.accelerations = accelerations;
    point.time_from_start = ros::Duration(time_from_start_);

    if (use_action_)
    {
      control_msgs::FollowJointTrajectoryGoal goal;
      goal.trajectory.header.stamp = ros::Time::now();
      goal.trajectory.header.frame_id = "base_link";
      goal.trajectory.joint_names = joint_names;
      goal.trajectory.points.push_back(point);

      traj_clients_[controller_name]->sendGoal(goal);
    }
    else
    {
      trajectory_msgs::JointTrajectory traj;
      traj.header.stamp = ros::Time::now();
      traj.header.frame_id = "base_link";
      traj.joint_names = joint_names;
      traj.points.push_back(point);
      
      traj_pubs_[controller_name].publish(traj);
    }
  }
}

/**
 * @brief Callback function for the topic joint_state
 *
 */
void JogFrameNodeAbs::joint_state_cb(sensor_msgs::JointStateConstPtr msg)
{
  // Check that the msg contains joints
  if (msg->name.empty() || msg->name.size() != msg->position.size())
  {
    ROS_WARN("Invalid JointState message");
    return;
  }
  // Update joint information
  for (int i=0; i<msg->name.size(); i++)
  {
    joint_map_[msg->name[i]] = msg->position[i];
  }
}


void JogFrameNodeAbs::resetInteractiveMarker()
{
  getFkPose();
  //ROS_WARN("req mutex 6");
  std::lock_guard<std::mutex> guard_pose_stamped(pose_stamped_mutex_);
  //ROS_INFO("mutex 6");
  geometry_msgs::Pose act_pose = pose_stamped_.pose;
  server_->setPose(int_marker_->name, pose_stamped_.pose);
  server_->applyChanges();
}

void JogFrameNodeAbs::interactiveMarkerFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  if(feedback != nullptr) {
    // reset the marker to the endeffectors position when mouse_up and stop the end effector
    // from moving by setting ref_msg_ to it's actual pose.
    if(feedback->event_type == feedback->MOUSE_UP) {
      
      //ROS_WARN("req mutex 7");
      std::lock_guard<std::mutex> guard_marker_msg(marker_msg_mutex_);
      //ROS_INFO("mutex 7");
      marker_msg_.header.stamp = ros::Time(0, 0);
      //resetInteractiveMarker();

      jog_msgs::JogFrame* reset_ref_msg = new jog_msgs::JogFrame();
      reset_ref_msg->header.stamp = ros::Time::now();
      reset_ref_msg->header.frame_id = frame_id_;
      reset_ref_msg->group_name = group_name_;
      reset_ref_msg->avoid_collisions = true;

      //ROS_WARN("req mutex 8");
      std::lock_guard<std::mutex> guard_pose_stamped(pose_stamped_mutex_);
      //ROS_INFO("mutex 8");
      reset_ref_msg->linear_abs.x = pose_stamped_.pose.position.x;
      reset_ref_msg->linear_abs.y = pose_stamped_.pose.position.y;
      reset_ref_msg->linear_abs.z = pose_stamped_.pose.position.z;
      reset_ref_msg->angular_abs.x = pose_stamped_.pose.orientation.x;
      reset_ref_msg->angular_abs.y = pose_stamped_.pose.orientation.y;
      reset_ref_msg->angular_abs.z = pose_stamped_.pose.orientation.z;

      //ROS_WARN("req mutex 9");
      std::lock_guard<std::mutex> guard_ref_msg(ref_msg_mutex_);
      geometry_msgs::Pose ref_pose;
      ref_pose.position.x = ref_msg_->linear_abs.x;
      ref_pose.position.y = ref_msg_->linear_abs.y;
      ref_pose.position.z = ref_msg_->linear_abs.z;
      server_->setPose(int_marker_->name, ref_pose);
      server_->applyChanges();

      //ROS_INFO("mutex 9");
      //ref_msg_ = jog_msgs::JogFrameConstPtr(reset_ref_msg);
    }
    else {
      //ROS_WARN("req mutex 10");
      std::lock_guard<std::mutex> guard_marker_msg(marker_msg_mutex_);
      //ROS_INFO("mutex 10");
      marker_msg_.header.stamp = ros::Time::now();
      marker_msg_.header.frame_id = frame_id_;
      marker_msg_.group_name = group_name_;
      marker_msg_.link_name = target_link_;
      marker_msg_.avoid_collisions = true;
      marker_msg_.linear_abs.x = feedback->pose.position.x;
      marker_msg_.linear_abs.y = feedback->pose.position.y;
      marker_msg_.linear_abs.z = feedback->pose.position.z;
      marker_msg_.angular_abs.x = feedback->pose.orientation.x;
      marker_msg_.angular_abs.y = feedback->pose.orientation.y;
      marker_msg_.angular_abs.z = feedback->pose.orientation.z;
    }
  }
}

void JogFrameNodeAbs::initInteractiveMarkers() 
{
  server_ = new interactive_markers::InteractiveMarkerServer("jog_frame_node_abs", "", true);

  // create an interactive marker for our server
  int_marker_ = new visualization_msgs::InteractiveMarker();
  int_marker_->header.frame_id = "base_link";
  //int_marker.header.stamp=ros::Time::now();
  int_marker_->name = "jog_frame_marker";
  int_marker_->description = "3-DOF Jogging Control";
  int_marker_->scale = 0.15;

  // create a marker
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = 0.075;
  marker.scale.y = 0.075;
  marker.scale.z = 0.075;
  marker.color.r = 1.0;
  marker.color.g = 0.5;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl marker_control;
  marker_control.always_visible = true;
  marker_control.markers.push_back( marker );
  marker_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D; 

  // add the control to the interactive marker
  int_marker_->controls.push_back( marker_control );

  // create a control which will move the box
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows
  visualization_msgs::InteractiveMarkerControl arrow_control;
  arrow_control.name = "move_x";
  arrow_control.orientation.w = 1;
  arrow_control.orientation.x = 1;
  arrow_control.orientation.y = 0;
  arrow_control.orientation.z = 0;
  arrow_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker_->controls.push_back(arrow_control);
  arrow_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker_->controls.push_back(arrow_control);

  arrow_control.name = "move_y";
  arrow_control.orientation.w = 1;
  arrow_control.orientation.x = 0;
  arrow_control.orientation.y = 1;
  arrow_control.orientation.z = 0;
  arrow_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker_->controls.push_back(arrow_control);
  arrow_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker_->controls.push_back(arrow_control);

  arrow_control.name = "move_z";
  arrow_control.orientation.w = 1;
  arrow_control.orientation.x = 0;
  arrow_control.orientation.y = 0;
  arrow_control.orientation.z = 1;
  arrow_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker_->controls.push_back(arrow_control);
  arrow_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker_->controls.push_back(arrow_control);

  // add the control to the interactive marker
  int_marker_->pose.position.z = 1;

  // add the interactive marker to our collection &
  // tell the server to call processMarkerFeedback() when feedback arrives for it
  server_->insert(*int_marker_);
  server_->setCallback(int_marker_->name, boost::bind(&JogFrameNodeAbs::interactiveMarkerFeedback, this, _1));
  server_->applyChanges();

  ROS_WARN_STREAM("Interactive Marker initialized: " << server_->size());
}

int JogFrameNodeAbs::get_controller_list()
{
  ros::NodeHandle gnh;

  // Get controller information from move_group/controller_list
  if (!gnh.hasParam("move_group/controller_list"))
  {
    ROS_ERROR_STREAM("move_group/controller_list is not specified.");
    return -1;
  }
  XmlRpc::XmlRpcValue controller_list;
  gnh.getParam("move_group/controller_list", controller_list);
  for (int i = 0; i < controller_list.size(); i++)
  {
    if (!controller_list[i].hasMember("name"))
    {
      ROS_ERROR("name must be specifed for each controller.");
      return -1;
    }
    if (!controller_list[i].hasMember("joints"))
    {
      ROS_ERROR("joints must be specifed for each controller.");
      return -1;
    }
    try
    {
      // get name member
      std::string name = std::string(controller_list[i]["name"]);
      // get action_ns member if exists
      std::string action_ns = std::string("");
      if (controller_list[i].hasMember("action_ns"))
      {
        action_ns = std::string(controller_list[i]["action_ns"]);
      }
      // get joints member
      if (controller_list[i]["joints"].getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_ERROR_STREAM("joints for controller " << name << " is not specified as an array");
        return -1;
      }
      auto joints = controller_list[i]["joints"];
      // Get type member
      std::string type = std::string("FollowJointTrajectory");
      if (!controller_list[i].hasMember("type"))
      {
        ROS_WARN_STREAM("type is not specifed for controller " << name << ", using default FollowJointTrajectory");
      }
      type = std::string(controller_list[i]["type"]);
      if (type != "FollowJointTrajectory")
      {
        ROS_ERROR_STREAM("controller type " << type << " is not supported");
        return -1;
      }
      // Create controller map
      cinfo_map_[name].action_ns = action_ns;
      cinfo_map_[name].joints.resize(joints.size());
      for (int j = 0; j < cinfo_map_[name].joints.size(); ++j)
      {
        cinfo_map_[name].joints[j] = std::string(joints[j]);
      }
    }
    catch (...)
    {
      ROS_ERROR_STREAM("Caught unknown exception while parsing controller information");
      return -1;
    }
  }
  return 0;
}

} // namespace jog_frame

/**
 * @brief Main function of the node
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "jog_frame_node_abs");
  jog_frame::JogFrameNodeAbs node;
  ros::Rate loop_rate(10);
  
  while ( ros::ok() )
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

