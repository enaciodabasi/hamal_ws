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

## 2.1 Project structure
The ROS workspace consists of several packages that make up the core functionality of the robot:


- **hamal_hardware** : This package consists of a hardware_interface class which reads the desirec values coming from the hardware using out EtherCAT Interface package and writes the commands coming from the ROS system. 
- **hamal_lifter_controller** : HamalLifterController plugin is a custom ROS controller that enables position control for the lifter. It can be interfaced via creating an action client for the "lifter_command_server"
- **hamal_control** : Contains the configuration and launch files for the DiffDriveController and HamalLifterController.
- **hamal_description** : Contains the URDF and mesh files of the robot.
- **hamal_localization** : Contains the configuration and launch files for the localization processes such as EKF.
- **hamal_mapping** : Contains configuration and launch files for mapping methods. The supported mapping algorithms are: GMapping, HectorSLAM and SLAM Toolbox.
- **hamal_navigation** : Contains the configuration and launch files for the move_base package.
- **hamal_bringup** : Using the "hamal_bringup.launch" script, the whole system can be activated.

## 2.2 Setting up the system

## 2.2.0 Setup
The hardware interface requires realtime privileges to run the EtherCAT communication loop in a realtime thread, therefore before opening the communication channels the following command must be executed as root:

```Bash
sudo su # Enter the password
ulimit -r 99 # This sets the realtime thread count.
```

The hamal_bringup package contains a script to setup the ROS environment, to use it create a symlink to the .sh file in the **hamal_bringup/scripts** directory inside the parent workspace directory for ease of use: 
```Bash
ln -s /home/$USER/hamal_ws/set_env /home/$USER/hamal_ws/src/hamal_bringup/scripts/set_env.sh

## Source the bash script with the required arguments
## Arguments can be seen with calling the script via the --help argument.
source set_env -m ${MASTER_IP} -w ${WORKSPACE_NAME}
```

Then the IP of the network interface that is connected to the scanners must be set to the scanners IP range:
```Bash
sudo ifconfig ${NETWORK_INTERFACE} 192.168.1.55
```
The IP can be different according to the scanners IP.
Ping the scanners to ensure they are connected:
```Bash
ping ${SCANNER_FRONT_IP}
ping ${SCANNER_REAR_IP}
```

When everything is set up, the whole system can be started by launching the bringup file:
```Bash
# Inside the root bash we used earlier
roslaunch hamal_bringup hamal_bringup.launch
```

## 2.2.1 Launch arguments

The main launch file is hamal_bringup.launch, which calls the individual launch files inside other packages.
The launch file can be called with the following arguments:

- **manual_enabled:** If manual mode is enabled, only the hardware, wheel and lifter controllers and the selected mapping nodes are launched. This parameter defaults to false, which in turn makes the launch file to launch the navigation package without any SLAM methods.
- **use_gmapping:** Launches the GMapping node.
- **use_hector:** Launches the Hector node.
- **use_slamtoolbox:** Launches SLAM toolbox with the selected mode
  - **mode:** SLAM toolbox modes: lifelong, localization, offline, online_async, online_sync
- **model:** AMRs URDF name.
- **map:** Path to the static map file. 

## 2.2.2 