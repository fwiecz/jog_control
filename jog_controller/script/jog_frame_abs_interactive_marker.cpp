#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    ROS_INFO_STREAM( feedback->marker_name << " is now at "
        << feedback->pose.position.x << ", " << feedback->pose.position.y
        << ", " << feedback->pose.position.z );
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "jog_abs_marker");
    ROS_WARN_STREAM("Namespace: " << ros::this_node::getNamespace());

    // create an interactive marker server on the topic namespace simple_marker
    interactive_markers::InteractiveMarkerServer server("jog_abs_marker");

    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    //int_marker.header.stamp=ros::Time::now();
    int_marker.name = "jog_abs_marker";
    int_marker.description = "3-DOF Jogging Control";
    int_marker.scale = 0.15;

    // create a grey box marker
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
    int_marker.controls.push_back( marker_control );

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
    int_marker.controls.push_back(arrow_control);

    arrow_control.name = "move_y";
    arrow_control.orientation.w = 1;
    arrow_control.orientation.x = 0;
    arrow_control.orientation.y = 1;
    arrow_control.orientation.z = 0;
    arrow_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(arrow_control);

    arrow_control.name = "move_z";
    arrow_control.orientation.w = 1;
    arrow_control.orientation.x = 0;
    arrow_control.orientation.y = 0;
    arrow_control.orientation.z = 1;
    arrow_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(arrow_control);

    // add the control to the interactive marker
    int_marker.pose.position.z = 1;

    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker, &processFeedback);

    // 'commit' changes and send to all clients
    server.applyChanges();

    // start the ROS main loop
    ros::spin();
}