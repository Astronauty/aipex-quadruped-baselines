## Code Structure Overview

Based on the [MIT Cheetah 3 Convex MPC Paper](https://dspace.mit.edu/bitstream/handle/1721.1/138000/convex_mpc_2fix.pdf)
<img width="850" height="722" alt="convex_mpc drawio" src="https://github.com/user-attachments/assets/653ca62d-42d8-4f11-9472-3d542dfe1611" />


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


There are now several unitree dependencies that we need to clone and setup. Clone all of the following repos under the /opt/unitree_robotics directory (necessary for cmake to properly link the packages). 

```bash
mkdir unitree_robotics
git clone https://github.com/unitreerobotics/unitree_sdk2.git &&
git clone https://github.com/unitreerobotics/unitree_ros2 && 
git clone https://github.com/unitreerobotics/unitree_mujoco
```

Within each of the cloned repos, follow the setup instructions from unitree in the respective README on github for each repo. We also provide instructions on setting up additional necessary dependencies under each repo.

1. Unitree SDK2
Follow the setup instructions under the README at: [https://github.com/unitreerobotics/unitree_sdk](https://github.com/unitreerobotics/unitree_sdk2)

```bash
cd unitree_sdk2
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
sudo make install
```

2. Unitree ROS2
Follow the setup instructions under the README at: [https://github.com/unitreerobotics/unitree_ros2]

Install the following additional dependencies for humble:
```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
sudo apt install ros-humble-rosidl-generator-dds-idl
sudo apt install python3-colcon-common-extensions
sudo apt install ros-humble-rosidl-generator-dds-idl
```

Remove sourcing of your default ROS workspace from .bashrc if it exists. Make sure that no active workspace is sourced before running the rest of the commands.

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

3. https://github.com/unitreerobotics/unitree_mujoco

```bash
sudo apt install libglfw3-dev libxinerama-dev libxcursor-dev libxi-dev libyaml-cpp-dev
```


The unitree_mujoco repo only publishes a subset of their /sportmodestate topic when simulating with mujoco. Our code relies on the topic for foot location information, so we need to modify /simulate/src/unitree_sdk2_bridge.cpp to broadcast foot positions from MuJoCo. Modify PublishHighState() method to the following:
```cpp
void UnitreeSdk2Bridge::PublishHighState()
{
    if (mj_data_ && have_frame_sensor_)
    {

        high_state_.position()[0] = mj_data_->sensordata[dim_motor_sensor_ + 10];
        high_state_.position()[1] = mj_data_->sensordata[dim_motor_sensor_ + 11];
        high_state_.position()[2] = mj_data_->sensordata[dim_motor_sensor_ + 12];

        high_state_.velocity()[0] = mj_data_->sensordata[dim_motor_sensor_ + 13];
        high_state_.velocity()[1] = mj_data_->sensordata[dim_motor_sensor_ + 14];
        high_state_.velocity()[2] = mj_data_->sensordata[dim_motor_sensor_ + 15];

        std::array<int, 4> foot_indices = {5, 9, 13, 17};
        
        // Custom foot position publish
        for (int i = 0; i < 4; i++)
        {
            high_state_.foot_position_body()[3*i + 0] = mj_data_->xpos[3*foot_indices[i] + 0];
            high_state_.foot_position_body()[3*i + 1] = mj_data_->xpos[3*foot_indices[i] + 1];
            high_state_.foot_position_body()[3*i + 2] = mj_data_->xpos[3*foot_indices[i] + 2];

            high_state_.foot_speed_body()[3*i + 0] = mj_data_->cvel[6*foot_indices[i] + 3]; // vx
            high_state_.foot_speed_body()[3*i + 1] = mj_data_->cvel[6*foot_indices[i] + 4]; // vy
            high_state_.foot_speed_body()[3*i + 2] = mj_data_->cvel[6*foot_indices[i] + 5]; // vz
        }

        high_state_puber_->Write(high_state_);
    }
}
```

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
