# Private simulator hardware transport. Public controllers use standard ROS interfaces.
uint8 HOLD=0
uint8 POSITION=1
uint8 VELOCITY=2
std_msgs/Header header
string run_id
string[] name
uint8[] mode
float64[] command
# Optional kinematic body resource; only supported by multicopter mode.
# linear.xyz in base_link, angular.z in rad/s; angular.xy must be zero.
bool body_velocity_control
geometry_msgs/Twist body_velocity
