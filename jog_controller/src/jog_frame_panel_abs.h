#ifndef JOG_FRAME_PANEL_ABS_H
#define JOG_FRAME_PANEL_ABS_H
#include <interactive_markers/interactive_marker_server.h>

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
        virtual void onInitialize();
        virtual void load(const rviz::Config& config);
        virtual void save(rviz::Config config) const;
        visualization_msgs::InteractiveMarker buildIntMarker();
    protected Q_SLOTS:
        void update();
    
    protected:
        QPushButton* enable_button_;
        QPushButton* reset_pose_button_;
        interactive_markers::InteractiveMarkerServer* m_server_;
        
};

}

#endif