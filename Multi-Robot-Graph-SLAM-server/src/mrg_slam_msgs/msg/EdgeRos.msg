# Edge of a graph

string uuid_str

string from_uuid_str
string to_uuid_str

uint8 TYPE_ANCHOR = 0
uint8 TYPE_ODOM = 1
uint8 TYPE_LOOP = 2
uint8 type 

geometry_msgs/Pose relative_pose

# Row-major representation of the 6x6 information matrix
float64[36] information