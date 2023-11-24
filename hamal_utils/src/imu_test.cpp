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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include <hamal_custom_interfaces/ImuTestDebugInfo.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_test_node");

    ros::NodeHandle nh;

    nav_msgs::Odometry odomFromController;
    nav_msgs::Odometry ekfOdomController;
    sensor_msgs::Imu imuMsg;

    auto controllerOdomSub = nh.subscribe<nav_msgs::Odometry>(
        "/hamal/mobile_base_controller/odom",
        1,
        [&](const nav_msgs::OdometryConstPtr& odom){
            odomFromController = *odom;
        }
    );

    auto ekfOdomSub = nh.subscribe<nav_msgs::Odometry>(
        "/odometry/filtered",
        1,
        [&](const nav_msgs::OdometryConstPtr& odom){
            ekfOdomController = *odom;
        }
    );

    auto imuSub = nh.subscribe<sensor_msgs::Imu>(
        "/imu/data",
        10,
        [&](const sensor_msgs::ImuConstPtr& imu_msg){
            imuMsg = *imu_msg;
        }
    );

    auto imuTestInfoPub = nh.advertise<hamal_custom_interfaces::ImuTestDebugInfo>(
        "/imu_test_info",
        10
    );

    std_msgs::Float64 linearAcc;
    ros::Time prevTime = ros::Time::now();
    double prevVelOdom, prevVelEkf = 0.0;
    ros::Rate rate(500.0);  
    while (ros::ok())
    {
        
        ros::spinOnce();
        
        hamal_custom_interfaces::ImuTestDebugInfo info;
        info.linear_acc_ekf = std::numeric_limits<double>::quiet_NaN();
        info.linear_acc_odom = std::numeric_limits<double>::quiet_NaN();

        double currVelOdom, currVelEkf = 0.0;
        
        double currAccOdom = currVelOdom - prevVelOdom;
        double currAccEkf = currVelEkf - prevVelEkf;
        
        if(abs(currAccOdom) >= 1e-4)
        {
            info.linear_acc_odom = currAccOdom;
        }
        if(abs(currAccEkf) >= 1e-4)
        {
            info.linear_acc_ekf = currAccEkf;
        }

        info.odom_diff_x= abs(odomFromController.pose.pose.position.x - ekfOdomController.pose.pose.position.x);
        tf2::Quaternion tfQuat;
        tf2::fromMsg(odomFromController.pose.pose.orientation, tfQuat);
        tf2::Matrix3x3 odomMat(tfQuat);
        double r, p, y = 0.0;
        odomMat.getRPY(r, p ,y);

        tf2::Quaternion tf;
        tf2::fromMsg(ekfOdomController.pose.pose.orientation, tf);
        tf2::Matrix3x3 ekfMat(tf);
        double r2, p2, y2 = 0.0;
        ekfMat.getRPY(r2, p2 ,y2);
        double yawDiff = abs(y - y2);

        info.odom_diff_yaw = yawDiff * (180.0 / M_PI);

        imuTestInfoPub.publish(info);

        prevVelOdom = currVelOdom;
        prevVelEkf = currVelEkf;

        ros::Time currTime = ros::Time::now();
        if((currTime - prevTime).toSec() >= 1.0){
            std::string infoStr = "##############################\n##############################\n";
            infoStr += "Current x displacement\nEncoder Odom: " + std::to_string(odomFromController.pose.pose.position.x);
            infoStr += " EKF Odom: " + std::to_string(ekfOdomController.pose.pose.position.x) + "\n";
            infoStr += "Current yaw\nEncoder Odom: " + std::to_string(y * (180.0 / M_PI)) + " | EKF Odom: " + std::to_string(y2 * (180.0 / M_PI));

            std::cout << infoStr << std::endl; 
            prevTime = currTime;
        }

        

        rate.sleep();
    }
    
    return 0;

}