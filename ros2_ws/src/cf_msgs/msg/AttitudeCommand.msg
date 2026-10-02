# Command from the MPC node to the safety supervisor.
# Angles follow ZYX Euler, world frame z up, body x forward, y left.
std_msgs/Header header
float64 roll         # rad, positive = right side down
float64 pitch        # rad, positive = nose down (accelerates towards +x)
float64 yaw_rate     # rad/s, positive = counter-clockwise seen from above
float64 thrust_cmd   # per-motor base thrust, 0..65535 (firmware units)
bool solver_ok       # false if the MPC solve failed
float64 solve_time   # s
float64 disturbance  # N, estimated unmodelled vertical force (offset-free MPC)
