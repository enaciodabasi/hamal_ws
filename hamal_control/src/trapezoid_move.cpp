/**
 * @file trapezoid_move.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <hamal_custom_interfaces/ProfileCommand.h>

#include <iostream>
#include <array>
#include <queue>

std::array<double ,3> calculateMotionDurations(const double control_var, double max_vel, double max_accel)
{
    double ta, tc, td = 0.0;

    ta = max_vel / max_accel;
    td = ta;

    double da = (0.5 * ta * max_vel);
    double dd = (0.5 * td * max_vel);

    double dc = abs(control_var) - abs(da + dd);

    tc = abs(dc / control_var);

    return std::array<double, 3>({ta, tc, td});

}

struct
{
    double max_vel;
    double max_acc;
    double target_position;
    double ta, tc, td;
    double pos_ref;
    double vel_ref;

    void reset(){
        target_position = 0.0;
        ta, tc, td = 0.0;
        pos_ref, vel_ref = 0.0;

    }
}goalInfo;

int main(int argc, char** argv)
{

    ros::init(argc, argv, "trapezoidal_motion_node");

    ros::NodeHandle nh;

    bool goalArrived, goalInProgress = false;
    ros::Rate rate(500.0);
    ros::Time profileStart, prevTime = ros::Time(0.0);

    std::queue<hamal_custom_interfaces::ProfileCommand> commandQueue;

    ros::Subscriber goalSub = nh.subscribe<hamal_custom_interfaces::ProfileCommand>(
        "/profile_command/trapezoidal",
        10,
        [&](const hamal_custom_interfaces::ProfileCommand::ConstPtr& profile_cmd){
            commandQueue.push(std::move(*profile_cmd));
            goalArrived = true;
        }
    );

    ros::Publisher cmdVelPub = nh.advertise<geometry_msgs::Twist>(
        "/hamal/mobile_base_controller/cmd_vel",
        1,
        false
    );

    while(ros::ok())
    {
        ros::spinOnce();
        
        
        //if(commandQueue.empty()){
        //    if(goalArrived)
        //        goalArrived = false;
        //    rate.sleep();
        //    continue;
        //}
        //else{
        //    goalArrived = true;
        //}

        if(goalArrived){
            
            auto currentGoal = commandQueue.front();
            goalInfo.target_position = currentGoal.target;
            goalInfo.max_vel = currentGoal.max_vel;
            goalInfo.max_acc = currentGoal.max_acc;
            commandQueue.pop();
            ROS_INFO("Starting manual profile generation.");
            auto profileTimes = calculateMotionDurations(goalInfo.target_position, goalInfo.max_vel, goalInfo.max_acc);
            goalInfo.ta = profileTimes[0];
            goalInfo.tc = profileTimes[1];
            goalInfo.td = profileTimes[2];
            ROS_INFO("Ta=%f | Tc=%f | Td=%f", goalInfo.ta, goalInfo.tc, goalInfo.td);
            ROS_INFO("Total profile time: %f seconds", (goalInfo.ta + goalInfo.tc + goalInfo.td));
            profileStart = ros::Time::now();
            prevTime = profileStart;

            goalInProgress = true;
            goalArrived = false;
            
        }
        else if(goalInProgress){

            ROS_INFO("Goal is active");
            auto currTime = ros::Time::now();
            auto timeSinceProfileStart = (currTime - profileStart).toSec();
            if(timeSinceProfileStart > (goalInfo.ta + goalInfo.tc + goalInfo.td)){
                
                ROS_INFO("Estimated profile duration exceeded by: %f. Resetting controller and braking.", timeSinceProfileStart);
                geometry_msgs::Twist cmdVel;
                cmdVel.angular.z = 0.0;
                cmdVel.linear.x = 0.0;
                cmdVelPub.publish(cmdVel);

                goalInfo.reset();
                goalInProgress = false;
                continue;
            }
            auto elaspedLoopTime = (currTime - prevTime).toSec();
            double velRef = 0.0;

            if(timeSinceProfileStart <= goalInfo.ta){
                velRef = goalInfo.vel_ref + (elaspedLoopTime * goalInfo.max_acc);
            }
            else if((timeSinceProfileStart > goalInfo.ta) && timeSinceProfileStart <= (goalInfo.ta + goalInfo.tc)){
                velRef = goalInfo.max_vel;
            }
            else if((timeSinceProfileStart >= goalInfo.ta + goalInfo.tc) && (timeSinceProfileStart <= goalInfo.ta + goalInfo.tc + goalInfo.td)){
                velRef = goalInfo.vel_ref - (elaspedLoopTime * goalInfo.max_acc);
            }

            geometry_msgs::Twist cmdVel;
            cmdVel.linear.x = velRef;
            ROS_INFO("Velocity reference: %f", velRef);
            cmdVelPub.publish(cmdVel);

        }


        rate.sleep();
    }

    return 0;
}