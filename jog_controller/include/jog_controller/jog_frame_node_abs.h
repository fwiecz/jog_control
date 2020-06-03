#ifndef JOG_FRAME_ABS_H
#define JOG_FRAME_ABS_H

#include <string>
#include <ros/ros.h>
#include <tf/tf.h>
#include <mutex>
#include <jog_msgs/JogFrameAbs.h>
#include <sensor_msgs/JointState.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <interactive_markers/interactive_marker_server.h>

typedef actionlib::SimpleActionClient < control_msgs::FollowJointTrajectoryAction > TrajClient;

namespace jog_frame
{

typedef struct
{
public:
  std::string action_ns;
  std::string type;
  std::vector<std::string> joints;
  
} Controller;

/**
 * Class JogFrameNodeAbs - Provides the jog_frame for absolute poses
 */
class JogFrameNodeAbs
{
public:
  /**
   * @breif: Default constructor for JogFrameNodeAbs Class.
   */
  JogFrameNodeAbs ();
  void jog_frame_cb (jog_msgs::JogFrameAbsConstPtr msg);
  void joint_state_cb (sensor_msgs::JointStateConstPtr msg);
  int get_controller_list();
  void update();
  void getFkPose();
  void jogStep();
  void publishPose(sensor_msgs::JointState state);
protected:
  ros::Subscriber joint_state_sub_, jog_frame_sub_;
  ros::ServiceClient fk_client_, ik_client_;

  std::map<std::string, Controller> cinfo_map_;
  std::map<std::string, TrajClient*> traj_clients_;
  std::map<std::string,ros::Publisher> traj_pubs_;

  std::map<std::string, double> joint_map_;
  geometry_msgs::PoseStamped pose_stamped_;
  std::mutex pose_stamped_mutex_;

  double time_from_start_;
  bool use_action_;
  bool intermittent_;

  jog_msgs::JogFrameAbsConstPtr ref_msg_;
  std::mutex ref_msg_mutex_;

  std::string target_link_;
  std::string group_name_;
  std::string frame_id_;
  bool avoid_collisions_;
  double damping_fac_;

  std::vector<std::string> exclude_joints_;
  sensor_msgs::JointState joint_state_;
  ros::Time last_stamp_;
};

} // namespace jog_frame

#endif // JOG_FRAME_NODE_H
