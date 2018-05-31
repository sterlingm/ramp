ramp_msgs/Path path
uint8 type 
bool print
ramp_msgs/BezierCurve[] bezierCurves
int8 segments
float64 max_speed_linear
float64 max_speed_angular

# For system-level test generation...
bool sl_traj
float64 sl_final_speed
duration sl_init_dur
duration sl_final_dur
