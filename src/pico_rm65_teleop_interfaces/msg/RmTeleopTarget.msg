std_msgs/Header header

uint32 source_seq

bool active
bool target_valid

string state
string reason

float64 pico_age_sec
float64 flange_age_sec

# Input stabilization diagnostics in the PICO tracking frame.
geometry_msgs/Pose raw_pico_pose
geometry_msgs/Pose filtered_pico_pose
string input_motion_state
bool still_hold_active
bool settling_active
float64 settling_remaining_translation_m
float64 settling_remaining_rotation_deg
float64 motion_window_translation_m
float64 motion_window_rotation_deg

geometry_msgs/Pose raw_target_pose
geometry_msgs/Pose limited_target_pose
