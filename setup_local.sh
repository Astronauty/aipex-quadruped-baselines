source /opt/ros/jazzy/setup.sh
source /opt/unitree_robotics/unitree_ros2/setup_local.sh

# Add Gurobi environment variables
export GUROBI_HOME=/opt/gurobi1202/linux64
export PATH=$PATH:$GUROBI_HOME/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$GUROBI_HOME/lib
export GRB_LICENSE_FILE="/home/daniel/gurobi.lic"
echo "GRB_LICENSE_FILE is set to: $GRB_LICENSE_FILE"