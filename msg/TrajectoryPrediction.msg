std_msgs/Header header

# Current estimated state of the volleyball in tracking_frame (e.g. base_link)
geometry_msgs/Point current_position
geometry_msgs/Vector3 current_velocity

# Predicted state at target interception plane / ground
geometry_msgs/Point landing_position
geometry_msgs/Vector3 landing_velocity
float32 time_to_landing

# Status flags
bool is_in_court
bool is_tracking
