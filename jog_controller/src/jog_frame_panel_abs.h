#ifndef JOG_FRAME_PANEL_ABS_H
#define JOG_FRAME_PANEL_ABS_H
#include <interactive_markers/interactive_marker_server.h>
#include <jog_msgs/JogFrameAbs.h>

#include <ros/ros.h>
#include <rviz/panel.h>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#  include <QtWidgets>
#else
#  include <QtGui>
#endif

namespace jog_controller
{
class JogFramePanelAbs : public rviz::Panel
{
    Q_OBJECT
    public:
        JogFramePanelAbs(QWidget* parent = 0);
        ~JogFramePanelAbs();
        QLayout* initUi(QWidget* parent);
        void updateFrame(QComboBox* frameCb);
        virtual void onInitialize();
        virtual void load(const rviz::Config& config);
        virtual void save(rviz::Config config) const;
        void hideEvent(QHideEvent* event);
        void showEvent(QShowEvent* event);
        void initInteractiveMarkers();
        void interactiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
        geometry_msgs::Pose* getTargetLinkPose();
        void resetInteractiveMarker();
    protected Q_SLOTS:
        void update();
        void respondOnOffCb(bool isChecked);
        void respondDamping(double value);
        void respondTargetLink(QString text);
        void respondGroupName(QString text);
        void respondFrameId(QString text);
        void respondCollision(bool isChecked);
    protected:
        QComboBox* frame_cb_;
        std::vector<std::string> group_names_;
        std::vector<std::string> link_names_;
        ros::Publisher jog_frame_abs_pub_;

        interactive_markers::InteractiveMarkerServer* server_;
        visualization_msgs::InteractiveMarker* int_marker_;
        jog_msgs::JogFrameAbs marker_msg_;
        bool on_publish_marker_;
        bool master_on_publish_;

        std::string target_link_;
        std::string group_name_;
        std::string frame_id_;
        bool avoid_collisions_;
        double damping_fac_;
};

}

#endif