/**
 * @file imu_test.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_test_node");

    ros::NodeHandle nh;

    nav_msgs::Odometry odomFromController;
    sensor_msgs::Imu imuMsg;

    auto odomSub = nh.subscribe<nav_msgs::Odometry>(
        "/hamal/mobile_base_controller/odom",
        10,
        [&](const nav_msgs::OdometryConstPtr& odom){
            odomFromController = *odom;
        }
    );

    auto imuSub = nh.subscribe<sensor_msgs::Imu>(
        "/imu/data",
        10,
        [&](const sensor_msgs::ImuConstPtr& imu_msg){
            imuMsg = *imu_msg;
        }
    );

    auto accPub = nh.advertise<std_msgs::Float64>(
        "/linear_acc_odom",
        10
    );

    std_msgs::Float64 linearAcc;

    double prevVel = 0.0;
    ros::Rate rate(500.0);  
    while (ros::ok())
    {
        
        ros::spinOnce();

        double currVel = 0.0;
        currVel = odomFromController.twist.twist.linear.x;
        double currAcc = currVel - prevVel;
        linearAcc.data = currAcc;
        
        accPub.publish(linearAcc);

        prevVel = currVel;

        rate.sleep();
    }
    

}