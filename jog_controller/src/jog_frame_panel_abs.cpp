#include <ros/package.h>
#include <ros/ros.h>

#include "jog_frame_panel_abs.h"
#include "jog_msgs/JogFrame.h"

#include <rviz/config.h>
#include <rviz/visualization_manager.h>
#include <tf/transform_listener.h>

namespace jog_controller 
{

JogFramePanelAbs::JogFramePanelAbs(QWidget* parent) : rviz::Panel(parent)
{
    ros::NodeHandle nh;
    // Get groups parameter of jog_frame_node
    nh.getParam("/jog_frame_node/group_names", group_names_);
    for (int i=0; i<group_names_.size(); i++)
    {
        ROS_INFO_STREAM("group_names:" << group_names_[i]);
    }
    nh.getParam("/jog_frame_node/link_names", link_names_);
    for (int i=0; i<link_names_.size(); i++)
    {
        ROS_INFO_STREAM("link_names:" << link_names_[i]);
    }

    QLayout* root_layout = initUi(parent);
    setLayout(root_layout);

    jog_frame_abs_pub_ = nh.advertise<jog_msgs::JogFrame>( "jog_frame", 1);
    master_on_publish_ = false;
    avoid_collisions_ = true;

    initInteractiveMarkers();
}

void JogFramePanelAbs::onInitialize() 
{
    connect( vis_manager_, SIGNAL( preUpdate() ), this, SLOT( update() ));
    updateFrame(frame_cb_);

    // initialize repeating timer
    QTimer* timer = new QTimer( this );
    connect( timer, &QTimer::timeout, this, QOverload<>::of(&JogFramePanelAbs::update));
    timer->start(100);

    resetInteractiveMarker();
}

/**
 * Gets repeatedley called by QTimer
 */
void JogFramePanelAbs::update()
{
    if(master_on_publish_ && on_publish_marker_) {
        jog_frame_abs_pub_.publish(marker_msg_);
    }
}

void JogFramePanelAbs::load(const rviz::Config& config)
{

}

void JogFramePanelAbs::save(rviz::Config config) const 
{

}

geometry_msgs::Pose* JogFramePanelAbs::getTargetLinkPose()
{
    std::shared_ptr<tf2_ros::Buffer> tf = vis_manager_->getTF2BufferPtr();
    try{
        geometry_msgs::Transform transform = tf->lookupTransform(frame_id_, target_link_, ros::Time(0)).transform;
        geometry_msgs::Pose* pose = new geometry_msgs::Pose();
        pose->position.x = transform.translation.x;
        pose->position.y = transform.translation.y;
        pose->position.z = transform.translation.z;
        pose->orientation.x = transform.rotation.x;
        pose->orientation.y = transform.rotation.y;
        pose->orientation.z = transform.rotation.z;
        pose->orientation.w = transform.rotation.w;
        return pose;
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }
    return nullptr;
}

void JogFramePanelAbs::resetInteractiveMarker()
{
    geometry_msgs::Pose* pose_ptr = getTargetLinkPose();
    if(pose_ptr != nullptr) {
        geometry_msgs::Pose markerPose = geometry_msgs::Pose(*pose_ptr);
        markerPose.position.x = markerPose.position.x;
        markerPose.position.y = markerPose.position.y;
        server_->setPose(int_marker_->name, markerPose);
        server_->applyChanges();
    }
}

void JogFramePanelAbs::interactiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
    if(feedback != nullptr) 
    {
        // reset the marker to the endeffectors position when mouse_up and stop the end effector
        // from moving by setting ref_msg_ to it's actual pose.
        if(feedback->event_type == feedback->MOUSE_UP) 
        {
            on_publish_marker_ = false;
            resetInteractiveMarker();
            geometry_msgs::Pose* pose = getTargetLinkPose();

            if(pose != nullptr) {
                // tell target link to stay where it is
                marker_msg_.header.stamp = ros::Time::now();
                marker_msg_.header.frame_id = frame_id_;
                marker_msg_.group_name = group_name_;
                marker_msg_.link_name = target_link_;
                marker_msg_.avoid_collisions = avoid_collisions_;
                marker_msg_.pose.position.x =  pose->position.x;
                marker_msg_.pose.position.y =  pose->position.y;
                marker_msg_.pose.position.z =  pose->position.z;
                marker_msg_.pose.orientation.x = pose->orientation.x;
                marker_msg_.pose.orientation.y = pose->orientation.y;
                marker_msg_.pose.orientation.z = pose->orientation.z;
                marker_msg_.pose.orientation.w = pose->orientation.w;
                marker_msg_.velocity_factor = velocity_fac_;
                
                if(master_on_publish_) {
                    jog_frame_abs_pub_.publish(marker_msg_);
                }
            }
        }
        else 
        {
            on_publish_marker_ = true;
            marker_msg_.header.stamp = ros::Time::now();
            marker_msg_.header.frame_id = frame_id_;
            marker_msg_.group_name = group_name_;
            marker_msg_.link_name = target_link_;
            marker_msg_.avoid_collisions = avoid_collisions_;
            // For some reason when dealing with markers x and y axis are mirrored
            marker_msg_.pose.position.x = feedback->pose.position.x;
            marker_msg_.pose.position.y = feedback->pose.position.y;
            marker_msg_.pose.position.z = feedback->pose.position.z;
            marker_msg_.pose.orientation.x = feedback->pose.orientation.x;
            marker_msg_.pose.orientation.y = feedback->pose.orientation.y;
            marker_msg_.pose.orientation.z = feedback->pose.orientation.z;
            marker_msg_.pose.orientation.w = feedback->pose.orientation.w;
            marker_msg_.velocity_factor = velocity_fac_;
        }
    }
}

void JogFramePanelAbs::respondOnOffCb(bool isChecked)
{
    master_on_publish_ = isChecked;
}

void JogFramePanelAbs::respondTargetLink(QString text)
{
    target_link_ = text.toStdString();
    resetInteractiveMarker();
}

void JogFramePanelAbs::respondGroupName(QString text)
{
    group_name_ = text.toStdString();
    resetInteractiveMarker();
}

void JogFramePanelAbs::respondFrameId(QString text)
{
    frame_id_ = text.toStdString();
    resetInteractiveMarker();
}


void JogFramePanelAbs::respondVelocity(double value)
{
    velocity_fac_ = value;
}

QLayout* JogFramePanelAbs::initUi(QWidget* parent)
{
    QTreeWidget* tree = new QTreeWidget();
    tree->setColumnCount(2);
    tree->setColumnWidth(0, 200);
    QStringList headers = {"Preferences", ""};
    tree->setHeaderLabels(headers);
    tree->setStyleSheet("QTreeView::item { border: 0px; margin: 2px; }");
    tree->setSelectionMode(QAbstractItemView::NoSelection);
    tree->setFocusPolicy(Qt::NoFocus);

    QStringList prefs = {"Enable Jogging", "Group", "Frame", "Target link", "Velocity Factor"};

    QList<QTreeWidgetItem *> items;
    for (int i = 0; i < prefs.size(); ++i) {
        QTreeWidgetItem* item = new QTreeWidgetItem((QTreeWidget*)0, QStringList(prefs[i]));
        items.append(item);
    }
    tree->insertTopLevelItems(0, items);

    QCheckBox* on_off_master_ = new QCheckBox();
    tree->setItemWidget(items.value(0), 1, on_off_master_);

    QComboBox* groupBox = new QComboBox();
    groupBox->setEditable(true);
    for (auto it = group_names_.begin(); it != group_names_.end(); it++) {
        const std::string& group = *it;
        if (group.empty())
            continue;
        groupBox->addItem(group.c_str());
    }
    group_name_ = groupBox->currentText().toStdString(); 
    tree->setItemWidget(items.value(1), 1, groupBox);

    frame_cb_ = new QComboBox();
    frame_cb_->setEditable(true);
    tree->setItemWidget(items.value(2), 1, frame_cb_);

    QComboBox* targetlink = new QComboBox();
    targetlink->setEditable(true);
    targetlink->clear();
    for (int i=0; i<link_names_.size(); i++) {
        targetlink->addItem(link_names_[i].c_str());
    }
    targetlink->setCurrentIndex(0);
    target_link_ = targetlink->currentText().toStdString();
    tree->setItemWidget(items.value(3), 1, targetlink);

    QDoubleSpinBox* speedSb = new QDoubleSpinBox();
    speedSb->setRange(0.1, 1.0);
    speedSb->setSingleStep(0.1);
    speedSb->setValue(0.6);
    velocity_fac_ = speedSb->value();
    tree->setItemWidget(items.value(4), 1, speedSb);

    QVBoxLayout* root_layout = new QVBoxLayout;
    root_layout->addWidget(tree);

    connect(on_off_master_, SIGNAL(toggled(bool)), this, SLOT(respondOnOffCb(bool)));
    connect(speedSb, SIGNAL(valueChanged(double)), this, SLOT(respondVelocity(double)));
    connect(targetlink, SIGNAL(currentTextChanged(QString)), this, SLOT(respondTargetLink(QString)));
    connect(groupBox, SIGNAL(currentTextChanged(QString)), this, SLOT(respondGroupName(QString)));
    connect(frame_cb_, SIGNAL(currentTextChanged(QString)), this, SLOT(respondFrameId(QString)));

    return root_layout;
}

void JogFramePanelAbs::updateFrame(QComboBox* frameCb)
{
    typedef std::vector<std::string> V_string;
    V_string frames;

    vis_manager_->getTF2BufferPtr()->_getFrameStrings( frames );
    std::sort(frames.begin(), frames.end());
    frameCb->clear();
    for (V_string::iterator it = frames.begin(); it != frames.end(); ++it )
    {
        const std::string& frame = *it;
        if (frame.empty())
            continue;
        frameCb->addItem(it->c_str());
    }
    frameCb->setCurrentIndex(0);
    frame_id_ = frameCb->currentText().toStdString();
}

void JogFramePanelAbs::initInteractiveMarkers() 
{
  server_ = new interactive_markers::InteractiveMarkerServer("jog_frame_node_abs", "", true);

  // create an interactive marker for our server
  int_marker_ = new visualization_msgs::InteractiveMarker();
  int_marker_->header.frame_id = "base_link";
  //int_marker.header.stamp=ros::Time::now();
  int_marker_->name = "jog_frame_marker";
  int_marker_->description = "6-DOF Jogging Control";
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
  server_->setCallback(int_marker_->name, boost::bind(&JogFramePanelAbs::interactiveMarkerFeedback, this, _1));
  server_->applyChanges();

  ROS_WARN_STREAM("Interactive Marker initialized: " << server_->size());
}

} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jog_controller::JogFramePanelAbs, rviz::Panel)
