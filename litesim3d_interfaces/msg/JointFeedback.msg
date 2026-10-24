string run_id
bool operational
sensor_msgs/JointState state
# Capability handshake and collision-accepted body state at state.header.stamp.
bool body_command_supported
geometry_msgs/Pose body_pose
geometry_msgs/Twist body_velocity
