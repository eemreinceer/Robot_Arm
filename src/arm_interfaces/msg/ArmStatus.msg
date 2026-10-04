# Robotik kolun genel durumu
std_msgs/Header header

# Durum sabitleri
uint8 IDLE=0
uint8 MOVING=1
uint8 GRASPING=2
uint8 PLACING=3
uint8 ERROR=4

uint8 state
string current_phase    # "idle", "pre_pick", "approaching", "grasping", ...
float32 progress        # 0.0 - 1.0
bool is_object_grasped
string error_message
float32[] joint_positions  # 6 eklem pozisyonu (radyan)
