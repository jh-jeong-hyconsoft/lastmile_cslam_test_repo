# Graph estimate, in contrast to GraphRos, the transforamtion and uncertainty of the edges is marginalized into the keyframes

std_msgs/Header header

string robot_name

string latest_keyframe_uuid_str                 # GID of latest keyframe

KeyFrameEstimate[] keyframes
EdgeEstimate[] edges