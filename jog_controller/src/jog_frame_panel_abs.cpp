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
}

QLayout* JogFramePanelAbs::initUi(QWidget* parent)
{
    QTreeWidget* tree = new QTreeWidget();
    tree->setColumnCount(2);
    QStringList headers = {"Preferences", ""};
    tree->setHeaderLabels(headers);
    //tree->headerItem()->setSizeHint(0 /*column index*/, QSize(50, 0) /*width*/);
    //tree->headerItem()->setSizeHint(1 /*column index*/, QSize(50, 0) /*width*/);
    //tree->headerItem()->setHidden(true);

    QStringList prefs = {"Enable Jogging", "Group", "Frame", "Target link", "Max Speed"};

    QList<QTreeWidgetItem *> items;
    for (int i = 0; i < prefs.size(); ++i) {
        QTreeWidgetItem* item = new QTreeWidgetItem((QTreeWidget*)0, QStringList(prefs[i]));
        items.append(item);
    }
    tree->insertTopLevelItems(0, items);

    QCheckBox* onOffJog = new QCheckBox();
    tree->setItemWidget(items.value(0), 1, onOffJog);

    QComboBox* groupBox = new QComboBox();
    for (auto it = group_names_.begin(); it != group_names_.end(); it++) {
        const std::string& group = *it;
        if (group.empty())
            continue;
        groupBox->addItem(group.c_str());
    }
    tree->setItemWidget(items.value(1), 1, groupBox);

    frame_cb_ = new QComboBox();
    tree->setItemWidget(items.value(2), 1, frame_cb_);

    QComboBox* targetlink = new QComboBox();
    targetlink->clear();
    for (int i=0; i<link_names_.size(); i++) {
        targetlink->addItem(link_names_[i].c_str());
    }
    targetlink->setCurrentIndex(0);
    target_link_id_ = targetlink->currentText().toStdString();
    tree->setItemWidget(items.value(3), 1, targetlink);

    QVBoxLayout* root_layout = new QVBoxLayout;
    root_layout->addWidget(tree);
    return root_layout;
}

void JogFramePanelAbs::updateFrame(QComboBox* frameCb)
{
    typedef std::vector<std::string> V_string;
    V_string frames;

    vis_manager_->getTFClient()->getFrameStrings( frames );
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

void JogFramePanelAbs::onInitialize() 
{
    connect( vis_manager_, SIGNAL( preUpdate() ), this, SLOT( update() ));
    updateFrame(frame_cb_); 
    //marker_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
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
