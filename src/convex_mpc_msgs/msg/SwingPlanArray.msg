# All swing plans currently held by the gait planner, plus the body state they
# were planned against. One message per plan cycle.

builtin_interfaces/Time stamp
float64 controller_time_s      # The planner's own clock, which the swing times use

geometry_msgs/Pose body_pose   # Measured body pose, world frame
geometry_msgs/Point body_velocity  # Measured body linear velocity, world frame

# Commanded reference, for comparison against the measured pose above.
geometry_msgs/Point reference_position
geometry_msgs/Point reference_velocity
float64 reference_yaw

SwingPlan[] plans

# Measured foot positions from BOTH available sources, so their disagreement is
# visible rather than inferred. See the foot-frame note: SportModeState's
# foot_position_body actually carries world-frame xpos of the *_foot body
# origin, while Pinocchio FK gives the foot about the CoM -- they differ by
# roughly the contact-sphere radius plus a residual.
geometry_msgs/Point[] feet_from_sportmode   # FL, FR, RL, RR
geometry_msgs/Point[] feet_from_fk          # FL, FR, RL, RR

# Which feet the gait schedule believes are in stance, same order.
bool[] stance

# MPC-solved ground reaction force per foot for the first horizon step, world
# frame, N (FL, FR, RL, RR). From the PREVIOUS plan cycle's solve (<= one plan
# period old), since this message is published before the solve. Lets a bag show
# whether the MPC asked for horizontal force, not just vertical.
geometry_msgs/Vector3[] grf_world
