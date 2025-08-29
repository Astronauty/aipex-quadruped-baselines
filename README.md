


## Dependency Installation
Tested on Ubuntu 22.04.05 with ROS Humble



### Eigen
https://eigen.tuxfamily.org/index.php?title=Main_Page#Download

Install eigen by clolning under /opt

```bash
cd /opt
git clone https://gitlab.com/libeigen/eigen.git
```

### ROS
https://docs.ros.org/en/humble/Installation.html

#### ROS Packages
```bash
sudo apt install ros-humble-joy
```

### Unitree
Make sure to install all of the following repos under the /opt/unitree_robotics directory.

```bash
mkdir unitree_robotics
git clone https://github.com/unitreerobotics/unitree_sdk2.git &&
git clone https://github.com/unitreerobotics/unitree_ros2 && 
git clone https://github.com/unitreerobotics/unitree_mujoco
```

Follow the instructions for each repo to build the unitree dependencies
1. [https://github.com/unitreerobotics/unitree_sdk](https://github.com/unitreerobotics/unitree_sdk2)

```bash
cd unitree_sdk2
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
sudo make install
```

2. https://github.com/unitreerobotics/unitree_ros2
```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
sudo apt install ros-humble-rosidl-generator-dds-idl
sudo apt install python3-colcon-common-extensions
sudo apt install ros-humble-rosidl-generator-dds-idl
```


Remove sourcing of the ros environment from .bashrc
```bash
sudo apt install gedit
sudo gedit ~/.bashrc
```

Compile cyclonedds
```bash
cd /opt/unitree_robotics/unitree_ros2/cyclonedds_ws/src
git clone https://github.com/ros2/rmw_cyclonedds -b humble
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x 
cd ..
colcon build --packages-select cyclonedds
```

```bash
source /opt/ros/humble/setup.bash 
colcon build
```

Update setup file to work with humble
```bash
gedit setup_local.sh
replace foxy with humble
```
replace source $HOME/unitree_ros2/cyclonedds_ws/install/setup.bash with source /opt/unitree_robotics/unitree_ros2/cyclonedds_ws/install/setup.bash

4. https://github.com/unitreerobotics/unitree_mujoco
sudo apt install libglfw3-dev libxinerama-dev libxcursor-dev libxi-dev libyaml-cpp-dev

Install MuJoCo
```bash
cd /opt
git clone https://github.com/google-deepmind/mujoco.git
cd /mujoco
git checkout 3.2.7
mkdir build && cd build
cmake ..
make -j4
sudo make install
```
```bash
cd /opt/unitree_robotics/unitree_mujoco/simulate
mkdir build && cd build
cmake ..
make -j4
```

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
```bash
source setup_local.sh
```
to source ROS and unitree ROS WS for use (i.e. with a simulator). 

Use 
```bash
source setup.sh
```
for deployment on the physical robot.
