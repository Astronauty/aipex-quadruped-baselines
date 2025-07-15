


## Dependency Installation
Tested on Ubuntu 22.04.05 Humble
ROS Jazzy

### Eigen
https://eigen.tuxfamily.org/index.php?title=Main_Page#Download

Install eigen by clolning under /opt

'''
cd /opt
git clone https://gitlab.com/libeigen/eigen.git
'''

### ROS
https://docs.ros.org/en/humble/Installation.html

### Unitree
Make sure to install all of the following repos under the /opt/unitree_robotics directory.

'''
mkdir unitree_robotics
git clone git@github.com:unitreerobotics/unitree_sdk2.git && 
git clone https://github.com/unitreerobotics/unitree_ros2 && 
git clone https://github.com/unitreerobotics/unitree_mujoco
'''

Follow the instructions for each repo to build the unitree dependencies
1. [https://github.com/unitreerobotics/unitree_sdk](https://github.com/unitreerobotics/unitree_sdk2)

mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
sudo make install

2. https://github.com/unitreerobotics/unitree_ros2
sudo apt install ros-humble-rmw-cyclonedds-cpp
sudo apt install ros-humble-rosidl-generator-dds-idl
sudo apt install python3-colcon-common-extensions


Remove sourcing of the ros environment from .bashrc
sudo apt install gedit
sudo gedit ~/.bashrc

Compile cyclonedds
cd /opt/unitree_robotics/unitree_ros2/cyclonedds_ws/src
git clone https://github.com/ros2/rmw_cyclonedds -b humble
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x 
cd ..
colcon build --packages-select cyclonedds

source /opt/ros/humble/setup.bash 
colcon build

4. https://github.com/unitreerobotics/unitree_mujoco




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

### Pinocchio
```bash
sudo apt install ros-$ROS_DISTRO-pinocchio
```

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
