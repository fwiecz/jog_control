#include <ros/package.h>
#include <QHBoxLayout>
#include <ros/ros.h>

#include "jog_frame_panel_abs.h"
#include "jog_msgs/JogFrame.h"

namespace jog_controller 
{

void markerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

JogFramePanelAbs::JogFramePanelAbs(QWidget* parent) : rviz::Panel(parent)
{
    QHBoxLayout* enable_layout = new QHBoxLayout;
    QLabel* test = new QLabel("test");
    enable_layout->addWidget(test);

    setLayout(enable_layout);
    
}

void JogFramePanelAbs::onInitialize() 
{
    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    m_server_ = new interactive_markers::InteractiveMarkerServer("jog_controller", "", true);
    
    visualization_msgs::InteractiveMarker m = JogFramePanelAbs::buildIntMarker();
    ROS_WARN_STREAM("nöa" << m.name);

    m_server_->insert(m, &markerFeedback);
    m_server_->applyChanges();
}

void JogFramePanelAbs::load(const rviz::Config& config)
{

}

void JogFramePanelAbs::save(rviz::Config config) const 
{

}

void JogFramePanelAbs::update()
{

}

void markerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    ROS_INFO_STREAM( feedback->marker_name << " is now at "
        << feedback->pose.position.x << ", " << feedback->pose.position.y
        << ", " << feedback->pose.position.z );
}

visualization_msgs::InteractiveMarker JogFramePanelAbs::buildIntMarker()
{
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "";
    int_marker.header.stamp=ros::Time::now();
    int_marker.name = "jog_abs_marker";
    int_marker.description = "6 DOF Marker for absolute jogging";
    
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.pose.position.z = 1;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back( marker );
    int_marker.controls.push_back( control );

    // create a control which will move the box
    // this control does not contain any markers,
    // which will cause RViz to insert two arrows
    visualization_msgs::InteractiveMarkerControl x_control;
    x_control.name = "move_x";
    x_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

    // add the control to the interactive marker
    int_marker.controls.push_back(x_control);
    return int_marker;
}

} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jog_controller::JogFramePanelAbs, rviz::Panel)
