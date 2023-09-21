# Hamal ROS Repo

# 1. Build And Installation

## 1.1 Building the EtherCAT Interface

### 1.1.1 Installing the Real Time Patch 

The real time patch varies from computer to computer. A good starting point can be found in the following link:

https://devicetests.com/install-real-time-patch-ubuntu-20-04-guide-beginners

### 1.1.2 Install IgH EtherCAT Master

The recommended steps to install the master can be found in the official documentation of IgH.

https://gitlab.com/etherlab.org/ethercat

### 1.1.3 Download the repository
Clone the project into the /home/${USER} directory.
```
git clone https://github.com/enaciodabasi/ethercat_interface
```
**Before building the project, make sure the library paths are correct in the main CMakeLists file.**

```CMake
set(
    etherlab_include
    /home/${USER}/ethercat/include
)

set(
    etherlab_lib
    /usr/local/lib/libethercat.so
)
```

The etherlab_include and etherlab_lib variables should be set to where they are installed on the computer.

Build the project using CMake:
```bash
cd ethercat_interface
cmake ..
mkdir build && cd build
```
## 1.2 Building the ROS workspace

Before building check the CMakeLists file in the hamal_hardware package to see if the path to the ethercat_interface library is correct.

```CMake
set(ethercat_interface_lib /usr/local/lib/libethercat_interface.so)
set(ethercat_interface_lib_INCLUDE_DIRS /home/${USER}/ethercat_interface/include/)
```

The build process is done with catkin:
```bash
cd hamal_ws
source /opt/ros/noetic/setup.bash
catkin_make
```

# 2. Using the software

The ROS workspace consists of several packages that make up the core functionality of the robot:


- **hamal_hardware** : This package consists of a hardware_interface class which reads the desirec values coming from the hardware using out EtherCAT Interface package and writes the commands coming from the ROS system. 
- **hamal_lifter_controller** : HamalLifterController plugin is a custom ROS controller that enables position control for the lifter. It can be interfaced via creating an action client for the "lifter_command_server"
- **hamal_control** : Contains the configuration and launch files for the DiffDriveController and HamalLifterController.
- **hamal_description** : Contains the URDF and mesh files of the robot.
- **hamal_localization** : Contains the configuration and launch files for the localization processes such as EKF.
- **hamal_mapping** : Contains configuration and launch files for mapping methods. The supported mapping algorithms are: GMapping, HectorSLAM and SLAM Toolbox.
- **hamal_navigation** : Contains the configuration and launch files for the move_base package.
- **hamal_bringup** : Using the "hamal_bringup.launch" script, the whole system can be activated.
