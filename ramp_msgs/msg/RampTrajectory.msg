Header header
uint16 id
trajectory_msgs/JointTrajectory trajectory
uint16[] i_knotPoints
ramp_msgs/BezierCurve[] curves
ramp_msgs/Path holonomic_path

bool feasible
float64 fitness

float64 min_obs_dis
float64 time_fitness
float64 orien_fitness
float64 obs_fitness

duration t_firstCollision
int8 i_subPopulation

duration t_start
