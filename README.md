


## Dependency Installation
Tested on Ubuntu 22.04.05 Humble
ROS Jazzy

### Eigen


### ROS
https://docs.ros.org/en/jazzy/Installation.html

### Unitree
Make sure to install all of the following repos under the /opt/unitree_robotics directory.

1. https://github.com/unitreerobotics/unitree_sdk
2. https://github.com/unitreerobotics/unitree_ros2
3. https://github.com/unitreerobotics/unitree_mujoco

### Gurobi Optimizer
The convex MPC controller requires Gurobi optimization solver:

1. Download and install Gurobi from https://www.gurobi.com/downloads/
2. Obtain a license (academic licenses are free for academic use)
3. Place your license file (typically named gurobi.lic) in your home directory
<!-- 4. Set the environment variable before running:
   ```bash
   export GRB_LICENSE_FILE="/home/daniel/gurobi.lic"
   ```
5. For persistent setup, add to your .bashrc:
   ```bash
   echo 'export GRB_LICENSE_FILE="/home/daniel/gurobi.lic"' >> ~/.bashrc
   source ~/.bashrc
   ``` -->


### Sourcing ROS and Unitree WS
Run
'''
source setup_local.sh
'''
to source ROS and unitree ROS WS for use (i.e. with a simulator). 

Use 
'''
source setup.sh
'''
for deployment on the physical robot.
