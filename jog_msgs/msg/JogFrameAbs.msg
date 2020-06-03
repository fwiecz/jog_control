# This is a message to hold data to jog by specifying a target
# frame. It uses MoveIt! kinematics, so you need to specify the
# JointGroup name to use in group_name. (lienar|angular)_delta is the
# amount of displacement.

# header message. You must set frame_id to define the reference
# coordinate system of the displacament
Header header

# Name of JointGroup of MoveIt!
string group_name

# Target link name to jog. The link must be in the JoingGroup
string link_name

# It uses avoid_collisions option of MoveIt! kinematics. If it is
# true, the robot doesn't move if any collisions occured.
bool avoid_collisions

# Target pose that should be followed by the target link
geometry_msgs/Pose pose

# The damping used for absolute jogging. It ranges from 0.1 to 1.0.
# It is cacluated by 'dist_to_goal * msg.damping_factor'
# resulting in faster movements when the target is far away and precise movements
# when the target is very close.
float64 damping_factor