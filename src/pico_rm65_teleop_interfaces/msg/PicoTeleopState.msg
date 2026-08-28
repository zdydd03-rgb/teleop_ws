std_msgs/Header header

uint32 protocol_version
uint32 seq
float64 source_timestamp_sec
uint16 target_hz

bool packet_valid
bool transport_alive
bool deadman
bool hand_teleop_enabled
bool teleop_armed
bool left_controller_connected
bool right_hand_tracked

string input_mode
string stop_reason
string receiver_reason

bool reset_origin
bool record_toggle

bool palm_valid
geometry_msgs/Pose palm_pose

bool wrist_valid
geometry_msgs/Pose wrist_pose
