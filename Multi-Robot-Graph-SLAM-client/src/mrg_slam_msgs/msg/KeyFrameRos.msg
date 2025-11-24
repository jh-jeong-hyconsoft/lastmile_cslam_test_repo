# KeyFrame of the graph

# unique boost uuid as string
string uuid_str 
string slam_uuid_str
string robot_name
int32 odom_counter 
float64 accum_distance
builtin_interfaces/Time stamp
bool first_keyframe
bool static_keyframe
geometry_msgs/Pose estimate
sensor_msgs/PointCloud2 cloud