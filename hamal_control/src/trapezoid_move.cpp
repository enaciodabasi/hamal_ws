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

#include "hamal_control/motion_profiles/trapezoidal_profile.hpp"

std::array<double ,3> calculateMotionDurations(const double control_var, double max_vel, double max_accel)
{
    double ta, tc, td = 0.0;

    ta = max_vel / max_accel;
    td = ta;

    double da = (0.5 * ta * max_vel);
    double dd = (0.5 * td * max_vel);

    double dc = abs(control_var) - abs(da + dd);

    tc = abs(dc / max_vel);

    return std::array<double, 3>({ta, tc, td});

}

double quaternionToDegree(const geometry_msgs::Quaternion& orientation_quaternion)
{
    tf2::Quaternion quat;
    tf2::fromMsg(orientation_quaternion, quat);
    tf2::Matrix3x3 m(quat);
    double r, p, y = 0.0;
    m.getRPY(r, p, y);

    return (y * (180.0 / M_PI));
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "trapezoidal_motion_node");

    ros::NodeHandle nh;

    bool goalArrived, goalInProgress = false;
    ros::Rate rate(500.0);
    ros::Time profileStart, prevTime = ros::Time(0.0);

    std::queue<hamal_custom_interfaces::ProfileCommand> commandQueue;
    
    double kp, ki, kd = 0.0;
    nh.param(
        "/profile_controller_params/trapezoidal_profile/velocity_controller/kp",
        kp,
        kp
    );

    nh.param(
        "/profile_controller_params/trapezoidal_profile/velocity_controller/ki",
        ki,
        ki
    );

    nh.param(
        "/profile_controller_params/trapezoidal_profile/velocity_controller/kd",
        kd,
        kd
    );

    ros::Subscriber goalSub = nh.subscribe<hamal_custom_interfaces::ProfileCommand>(
        "/profile_command/trapezoidal",
        10,
        [&](const hamal_custom_interfaces::ProfileCommand::ConstPtr& profile_cmd){
            commandQueue.push(std::move(*profile_cmd));
            goalArrived = true;
        }
    );

    nav_msgs::Odometry currentOdom;

    ros::Subscriber odomSub = nh.subscribe<nav_msgs::Odometry>(
        "/hamal/mobile_base_controller/odom",
        10,
        [&](const nav_msgs::Odometry::ConstPtr& odom_msg){
            currentOdom = (*odom_msg);
        }   
    );

    std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Twist>> cmdVelRtPub = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::Twist>>(
        nh,
        "/hamal/mobile_base_controller/cmd_vel",
        1,
        false
    );

    /* ros::Publisher cmdVelPub = nh.advertise<geometry_msgs::Twist>(
        "/hamal/mobile_base_controller/cmd_vel",
        1,
        false
    ); */

    ros::Publisher controlledCmdVelPub = nh.advertise<geometry_msgs::TwistStamped>(
        "/hamal/mobile_base_controller/controlled_cmd_vel",
        1,
        false
    );

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener baseOdomTfListener(tfBuffer, nh, true);

    PID velController;
    ROS_INFO("Velocity PID params: Kp: %f | Ki: %f | Kd: %f", kp, ki, kd);
    velController.setParams(kp, ki, kd);

    PID linearMoveOrientationController;
    kp, ki, kd = 0.0;
    nh.param(
        "/profile_controller_params/trapezoidal_profile/orientation_controller/kp",
        kp,
        kp
    );

    nh.param(
        "/profile_controller_params/trapezoidal_profile/orientation_controller/ki",
        ki,
        ki
    );

    nh.param(
        "/profile_controller_params/trapezoidal_profile/orientation_controller/kd",
        kd,
        kd
    );

    linearMoveOrientationController.setParams(kp, ki, kd);

    double orientationSetPoint = 0.0;
    bool controlOrientation = true; 
    double distanceWithoutFeedback = 0.0;
    double distanceWithControlledVelReference = 0.0;
    double realDistanceTraveled = 0.0;
    double prevX = currentOdom.pose.pose.position.x;
    double realDistanceTraveledAngular = abs(quaternionToDegree(currentOdom.pose.pose.orientation));
    
    auto resetVars = [&]() -> void {
        geometry_msgs::Twist cmdVel;
                cmdVel.angular.z = 0.0;
                cmdVel.linear.x = 0.0;

        if(cmdVelRtPub->trylock()){
            cmdVelRtPub->msg_ = cmdVel;
            cmdVelRtPub->unlockAndPublish();
        };    
        ROS_INFO("Distance traveled via commands: %f", distanceWithoutFeedback);
        distanceWithoutFeedback = 0.0;
        orientationSetPoint = 0.0;
        goalInfo.reset();
        goalInProgress = false;
        distanceWithControlledVelReference = 0.0;
    };

    while(ros::ok())
    {
        ros::spinOnce();
        

        if(commandQueue.empty()){
            if(goalArrived)
                goalArrived = false;
        }
        else{
            if(!goalInProgress)
                goalArrived = true;
        }

        if(goalArrived){
            
            auto currentGoal = commandQueue.front();
            goalInfo.target_position = currentGoal.target;
            goalInfo.max_vel = currentGoal.max_vel;
            goalInfo.max_acc = currentGoal.max_acc;
            if(currentGoal.linear){
                goalInfo.motion_type = MotionType::Linear;
                std::optional<geometry_msgs::TransformStamped> currentTfOpt;
                try{
                    geometry_msgs::TransformStamped currentTf = tfBuffer.lookupTransform(
                        "odom",
                        "base_link",
                        ros::Time(0.0)
                    );

                    currentTfOpt = currentTf;
                }catch(...){
                    controlOrientation = false;
                }

                if(currentTfOpt){
                    auto orientationQuat = currentTfOpt.value().transform.rotation;
                    orientationSetPoint = quaternionToDegree(orientationQuat);
                }
            }
            else{
                goalInfo.motion_type = MotionType::Angular;
                orientationSetPoint = 0.0;
            }
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
            realDistanceTraveled = 0.0;
            realDistanceTraveledAngular = quaternionToDegree(currentOdom.pose.pose.orientation);
            prevX = 0.0;
            distanceWithoutFeedback = 0.0;
            goalInProgress = true;
            goalArrived = false;
            distanceWithControlledVelReference = 0.0;
            orientationSetPoint = 0.0;
            
        }
        else if(goalInProgress){

            auto currTime = ros::Time::now();
            auto timeSinceProfileStart = (currTime - profileStart).toSec();
            if(timeSinceProfileStart > (goalInfo.ta + goalInfo.tc + goalInfo.td)){
                
                ROS_INFO("Estimated profile duration exceeded by: %f. Resetting controller and braking.", timeSinceProfileStart);
                resetVars();
                /* geometry_msgs::Twist cmdVel;
                cmdVel.angular.z = 0.0;
                cmdVel.linear.x = 0.0;

                if(cmdVelRtPub->trylock()){
                    cmdVelRtPub->msg_ = cmdVel;
                    cmdVelRtPub->unlockAndPublish();
                }

                ROS_INFO("Distance traveled via commands: %f", distanceWithoutFeedback);

                distanceWithoutFeedback = 0.0;
                goalInfo.reset();
                goalInProgress = false; */
                continue;
            }
            auto elaspedLoopTime = (currTime - prevTime).toSec();
            prevTime = currTime;
            
            double velRef = 0.0;
            double currentValue = 0.0;
            std::optional<geometry_msgs::TransformStamped> orientTfOpt;
            switch (goalInfo.motion_type)
            {
            case MotionType::Linear:
                currentValue = currentOdom.twist.twist.linear.x;
                realDistanceTraveled += currentValue * elaspedLoopTime;
                if(controlOrientation){
                    try{
                        orientTfOpt = tfBuffer.lookupTransform("odom", "base_link", ros::Time(0.0));
                    }catch(...){
                        controlOrientation = false;
                    }
                }
            case MotionType::Angular:
                currentValue = currentOdom.twist.twist.angular.z;
                realDistanceTraveledAngular += currentValue * elaspedLoopTime;
                orientTfOpt = std::nullopt;
                break;
            default:
                currentValue = 0.0;
                orientTfOpt = std::nullopt;
                break;
            }
            
            if(timeSinceProfileStart <= goalInfo.ta){
                velRef = goalInfo.vel_ref + (elaspedLoopTime * goalInfo.max_acc);
            }
            else if((timeSinceProfileStart > goalInfo.ta) && timeSinceProfileStart <= (goalInfo.ta + goalInfo.tc)){
                velRef = goalInfo.max_vel;
            }
            else if((timeSinceProfileStart >= goalInfo.ta + goalInfo.tc) && (timeSinceProfileStart <= goalInfo.ta + goalInfo.tc + goalInfo.td)){
                velRef = goalInfo.vel_ref - (elaspedLoopTime * goalInfo.max_acc);
            }

            if(orientTfOpt && controlOrientation){
                const auto orientTf = orientTfOpt.value();
                double orientCtrl = linearMoveOrientationController.control(orientationSetPoint, quaternionToDegree(orientTf.transform.rotation), currTime);
                goalInfo.angular_vel_ref = orientCtrl * elaspedLoopTime;
                double err = quaternionToDegree(orientTf.transform.rotation) - orientationSetPoint;
                ROS_INFO("Orientation error: %f | Angular velocity to control: %f", err, goalInfo.angular_vel_ref);
            }else{ROS_WARN("Not controlling orientation.");}
            
            distanceWithoutFeedback += velRef * elaspedLoopTime;
            auto controlledRef = velController.control(currentValue, goalInfo.vel_ref, currTime);
            
            ROS_INFO("Distance traveled (feedback): %f | Distance traveled (open-loop): %f", realDistanceTraveled, distanceWithoutFeedback);
            if(goalInfo.target_position < 0){
                velRef = velRef * -1.0;
            }
            goalInfo.vel_ref = velRef;
            geometry_msgs::Twist cmdVel;
            
            cmdVel.linear.x = velRef;
            /* cmdVel.angular.z = goalInfo.angular_vel_ref; */
            controlledRef += velRef;
            distanceWithControlledVelReference += controlledRef * elaspedLoopTime;


            ROS_INFO("Distance traveled (controlled): %f", distanceWithControlledVelReference);
            
            ROS_INFO("Velocity Reference: %f | Controlled reference: %f", velRef, controlledRef);
            if(cmdVelRtPub->trylock()){
                    cmdVelRtPub->msg_ = cmdVel;
                    cmdVelRtPub->unlockAndPublish();
            }

            geometry_msgs::TwistStamped controllerCmdVel;
            controllerCmdVel.twist.linear.x = controlledRef;
            controlledCmdVelPub.publish(controllerCmdVel);
        }


        rate.sleep();
    }

    return 0;
}