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
    ROS_WARN("Got command: x: %f [m] | yaw: %f[deg]", cmd.x, cmd.degree);
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

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped baseLinkToMapTransform;

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

    double targetYaw = 0.0;
    double targetDistanceToTravelInX = 0.0;
    double startX = 0.0;
    
    while(ros::ok())
    {
        

        if(commandPtr){
            
            baseLinkToMapTransform = tfBuffer.lookupTransform(
                "map",
                "base_link",
                ros::Time(0),
                ros::Duration(0.5)
            );

            move_base_msgs::MoveBaseGoal goal;
            
            const double radCmd = degToRad(commandPtr->degree);
            
            if(radCmd != 0){
                tf2::Quaternion quat;
                quat.setRPY(0.0, 0.0, radCmd);
                geometry_msgs::Quaternion quatMsg;
                quatMsg = tf2::toMsg(quat);
                goal.target_pose.pose.orientation = quatMsg;
            }
            else{
                goal.target_pose.pose.orientation.x = 0.0;
                goal.target_pose.pose.orientation.y = 0.0;
                goal.target_pose.pose.orientation.z = 0.0;
                goal.target_pose.pose.orientation.w = 1.0;
            }
         
            //goal.target_pose.pose.orientation.x = 0.0;
            //goal.target_pose.pose.orientation.y = 0.0;
            //goal.target_pose.pose.orientation.z = 0.0;
            //goal.target_pose.pose.orientation.w = 1.0;   
            goal.target_pose.header.frame_id = "base_link";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = commandPtr->x;
            goal.target_pose.pose.position.y = 0.0;
            goal.target_pose.pose.position.z = 0.0;

            auto robotPose = goal.target_pose;
            tf2::doTransform(robotPose, robotPose, baseLinkToMapTransform);
            goal.target_pose = robotPose;
            goal.target_pose.header.frame_id = "map";
            targetYaw = commandPtr->degree;
            startX = odomMsg.pose.pose.position.x;
            targetDistanceToTravelInX = commandPtr->x;
            //startX = commandPtr->x;
            client.sendGoal(goal);
            delete commandPtr;
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