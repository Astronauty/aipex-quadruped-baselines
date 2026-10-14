# One planned swing-leg trajectory, as the gait planner built it.
#
# Published by the controller at the plan rate so a visualiser can draw the
# spline the swing controller is actually tracking. Everything is WORLD frame,
# matching the swing pipeline, so a frame error shows up as wrong geometry on
# screen rather than being hidden by a transform.

string leg              # "FL" | "FR" | "RL" | "RR"

float64 start_time_s    # Lift-off time (controller clock)
float64 end_time_s      # Touchdown time (controller clock)
float64 swing_height_m  # Apex height the spline was built with

geometry_msgs/Point start_position   # Lift-off point, world frame
geometry_msgs/Point end_position     # Touchdown target, world frame

# The spline sampled uniformly over [start_time_s, end_time_s]. Sampling in the
# publisher keeps the spline maths in one place -- a visualiser that re-derived
# the curve could draw something the controller never tracked.
geometry_msgs/Point[] sampled_points

# Where the reference says the foot should be right now, and where it measurably
# is. These two being far apart is the swing-tracking error made visible.
geometry_msgs/Point reference_position
geometry_msgs/Point measured_position
bool in_flight          # True while start_time_s < now < end_time_s
