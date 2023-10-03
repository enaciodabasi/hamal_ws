/**
 * @file orientation_test.cpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-08-31
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "hamal_control/orientation_test.hpp"

double cmd = 0.0;
bool newCmd = false;

hamal_custom_interfaces::ManualMoveCommand* commandPtr = nullptr;

nav_msgs::Odometry odomMsg;

void getInputDegree(const std_msgs::Float64::ConstPtr& degree_command)
{
    newCmd = true;
    cmd = degree_command->data;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    odomMsg = *odom_msg;
}

void manualMoveCommandCallback(const hamal_custom_interfaces::ManualMoveCommand::ConstPtr& command)
{
    if(!commandPtr){
        commandPtr = new hamal_custom_interfaces::ManualMoveCommand();
    }

    auto& cmd = *commandPtr;
    cmd.degree = command->degree;
    cmd.x = command->x;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "orientation_test_node");

    ros::NodeHandle nh;

    ros::Subscriber degreeSub;
    degreeSub = nh.subscribe("/degree_command", 10, getInputDegree);

    ros::Subscriber odomSub;
    odomSub = nh.subscribe("/hamal/mobile_base_controller/odom", 10, odomCallback);

    ros::Subscriber cmdSub;
    cmdSub = nh.subscribe("/manual_move", 10, manualMoveCommandCallback);

    ros::Rate rate(50.0); // Run at 50 Hz.
    
    MoveBaseClient client("move_base", true);
    
    /* while(!client.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for move_base action server.");
    }
    
    if(!client.isServerConnected()){
        ROS_ERROR("Could not connect to the move_base action server.");
        return -1;
    } */
    
    while(ros::ok())
    {
        double targetYaw = 0.0;
        double targetDistanceToTravelInX = 0.0;
        double startX = 0.0;

        if(commandPtr){


            const double radCmd = degToRad(commandPtr->degree);
            tf2::Quaternion quat;
            quat.setRPY(0.0, 0.0, radCmd);
            geometry_msgs::Quaternion quatMsg;
            quatMsg = tf2::toMsg(quat);
            
            move_base_msgs::MoveBaseGoal goal;

            goal.target_pose.pose.orientation = quatMsg;
            goal.target_pose.header.frame_id = "base_link";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = commandPtr->x;
            goal.target_pose.pose.position.y = 0.0;
            goal.target_pose.pose.position.z = 0.0;
            targetYaw = commandPtr->degree;
            startX = odomMsg.pose.pose.position.x;
            targetDistanceToTravelInX = commandPtr->x;
            //startX = commandPtr->x;
            commandPtr = nullptr;
        }

        const geometry_msgs::Quaternion currQuat = odomMsg.pose.pose.orientation;
        tf2::Quaternion tfQuat;
        tf2::fromMsg(currQuat, tfQuat);

        tf2::Matrix3x3 m(tfQuat);
        double r, p, y = 0.0;
        m.getRPY(r, p, y);
        double xDiff =  odomMsg.pose.pose.position.x - startX; // Change in x coordinates in meters.
        ROS_INFO("Distance to travel: %f | Current traveled: %f \nCurrent Yaw: %f | Target Yaw: ", targetDistanceToTravelInX, xDiff, radToDeg(y), targetYaw);
        /* if(newCmd){
            ROS_INFO("New degree command received.");
            newCmd = false;

            move_base_msgs::MoveBaseGoal goal;
            const double radCmd = degToRad(cmd);

            tf2::Quaternion quat;
            quat.setRPY(0.0, 0.0, radCmd);

            geometry_msgs::Quaternion quatMsg;
            quatMsg = tf2::toMsg(quat);
            ROS_INFO("Target yaw: %f", radCmd);

            goal.target_pose.pose.orientation = quatMsg;
            goal.target_pose.header.frame_id = "base_link";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = 0.0;
            goal.target_pose.pose.position.y = 0.0;
            goal.target_pose.pose.position.z = 0.0;

            client.sendGoal(goal);

        }

        const geometry_msgs::Quaternion currQuat = odomMsg.pose.pose.orientation;
        tf2::Quaternion tfQuat;
        tf2::fromMsg(currQuat, tfQuat);

        tf2::Matrix3x3 m(tfQuat);
        double r, p, y = 0.0;
        m.getRPY(r, p, y);

        std::string yawStr = std::to_string(y);

        ROS_INFO("Yaw Value: %f", radToDeg(y)); */

        ros::spinOnce();
        rate.sleep();
    }
    

    ros::shutdown();

    return 0;

}