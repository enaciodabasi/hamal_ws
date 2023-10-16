#! /usr/bin/bash
##### Help function #####
Help()
{
    # Display Help
    echo "Set up the ROS environment for the PC."
    echo
    echo "Syntax: ./set_env.sh [-m|h|w]"
    echo "options:"
    echo "m     ROS Master IP"
    echo "hn     Hostname of the ROS Master PC (optional if the /etc/hosts file is already configured)"
    echo "w     Name of the ROS workspace."
    echo
} 

while getopts :h:m:hn:w: arg
do
    case "${arg}" in
        h) Help exit;;
        m) master_ip=${OPTARG};;
        hn) master_hostname=${OPTARG};;
        w) workspace_name=${OPTARG};;
    esac 
done

## Check if master's IP address is provided in the arguments:
## If not exit with error.
if [ -z ${master_ip+x} ]; 
then 
    
    echo "No master ip provided.";
    exit -1;
fi

ros_distro=$(echo $ROS_DISTRO)

if [ -z "$ros_distro" ];
then
    source /opt/ros/noetic/setup.bash
fi

username=$(echo $USER)
workspace_path=/home/${username}/${workspace_name}

source ${workspace_path}/devel/setup.bash

ip_self=$(hostname -I)


export ROS_MASTER_URI=http://${master_ip}:11311
export ROS_IP=${ip_self}