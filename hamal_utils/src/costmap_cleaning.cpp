/**
 * @file costmap_cleaning.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-12-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "hamal_utils/costmap_cleaning.hpp"

int main(int argc, char** argv)
{

    ros::init(argc, argv, "costmap_cleaning_node");

    ros::NodeHandle nh;

    double currLinearVel, currAngularVel = 0.0;
    ros::Subscriber cmdVelSub = nh.subscribe<nav_msgs::Odometry>(
        "/hamal/mobile_base_controller/odom", 
        1,
        [&](const nav_msgs::OdometryConstPtr& odom){
            currAngularVel = odom->twist.twist.angular.z,
            currLinearVel = odom->twist.twist.linear.x;
        }    
    ); 
    
    actionlib_msgs::GoalStatus currGoalStatus;
    ros::Subscriber moveBaseFeedbackSub = nh.subscribe<move_base_msgs::MoveBaseActionFeedback>(
        "/move_base/feedback",
        10,
        [&](const move_base_msgs::MoveBaseActionFeedback::ConstPtr& move_base_feedback){

            currGoalStatus = move_base_feedback->status;
        }
    );

    bool dockingActivated = false;
    ros::Subscriber dockingSignalSub = nh.subscribe<std_msgs::Empty>(
        "/docking_signal",
        1,
        [&](const std_msgs::Empty::ConstPtr& docking_signal){
            dockingActivated = true;
        }
    ); 

    ros::ServiceClient clearCostmapClient = nh.serviceClient<std_srvs::Empty>(
        "/move_base/clear_costmaps",
        false
    );
    
    ros::Time prevUpdateTime = ros::Time::now();

    double loopHz = 50.0;
    nh.param("/move_base/local_costmap/update_frequency", loopHz, loopHz);

    ros::Rate rate(loopHz);
    bool shouldClear = false;
    while(ros::ok())
    {
        ros::spinOnce();
        
        if(currGoalStatus.status == actionlib_msgs::GoalStatus::ACTIVE){
            if (currLinearVel <= 1.0e-6 || currAngularVel <= 1.0e-6)
            {   
                auto currTime = ros::Time::now();
                double elapsed = (currTime - prevUpdateTime).toSec();
                if(elapsed >= 5.0){
                    prevUpdateTime = currTime;  
                    shouldClear = true;
                }
                    
            }
        }
        else if( 
            /* currGoalStatus.status == actionlib_msgs::GoalStatus::PENDING || */ 
            currGoalStatus.status == actionlib_msgs::GoalStatus::REJECTED ||
            currGoalStatus.status == actionlib_msgs::GoalStatus::LOST
        )
        {   
            ROS_INFO("Robot stuck.");
            shouldClear = true;
        }
        else if(dockingActivated){
            shouldClear = true;
            dockingActivated = false;
        }


        if(shouldClear){

            std_srvs::Empty::Request srvReq;
            std_srvs::Empty::Response srvRep;
            bool cleared = clearCostmapClient.call(srvReq, srvRep);
            ROS_INFO("Clearing costmaps.");
            shouldClear = false;
        }

        rate.sleep();
    }

    return 0;
}