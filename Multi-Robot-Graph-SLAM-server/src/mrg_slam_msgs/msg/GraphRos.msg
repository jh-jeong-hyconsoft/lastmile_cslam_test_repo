# Graph

std_msgs/Header header

string robot_name

string latest_keyframe_uuid_str
geometry_msgs/Pose latest_keyframe_odom     # odometry transform of latest keyframe, used to calc odom to map transform for that robot

KeyFrameRos[] keyframes
EdgeRos[] edges