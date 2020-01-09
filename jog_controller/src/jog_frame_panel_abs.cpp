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
    QCheckBox* onOffButton = new QCheckBox("Enable Jogging");
    enable_layout->addWidget(onOffButton);

    setLayout(enable_layout);
}

void JogFramePanelAbs::onInitialize() 
{
    //ros::NodeHandle nh;
    //ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
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

} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jog_controller::JogFramePanelAbs, rviz::Panel)
