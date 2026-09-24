source /opt/ros/humble/setup.sh
source /opt/unitree_robotics/unitree_ros2/setup_local.sh
source install/local_setup.sh


# Must match domain_id in unitree_mujoco/simulate/config.yaml (currently 0),
# otherwise the controller cannot see /lowstate or /sportmodestate.
export ROS_DOMAIN_ID=0

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/install/local_setup.sh"
# source "$(pwd)/install/local_setup.sh"


# Must match domain_id in unitree_mujoco/simulate/config.yaml (currently 0),
# otherwise the controller cannot see /lowstate or /sportmodestate.
export ROS_DOMAIN_ID=0

# Write ROS node logs into the repo rather than ~/.ros/log, so a run's output
# sits next to the code and config that produced it. Note this is NOT the top
# level log/ directory, which belongs to colcon's build logs.
export ROS_LOG_DIR="$SCRIPT_DIR/ros_logs"
mkdir -p "$ROS_LOG_DIR"

# Add Gurobi environment variables
export GUROBI_HOME=/opt/gurobi1202/linux64
export PATH=$PATH:$GUROBI_HOME/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$GUROBI_HOME/lib
export GRB_LICENSE_FILE="/opt/gurobi1202/gurobi.lic"
echo "GRB_LICENSE_FILE is set to: $GRB_LICENSE_FILE"
