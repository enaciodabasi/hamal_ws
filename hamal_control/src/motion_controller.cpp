/**
 * @file motion_controller.cpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief
 * @version 0.1
 * @date 2024-02-13
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "hamal_control/motion_controller/motion_controller.hpp"
#include <diff_drive_controller_hamal/DiffDriveControllerHamalConfig.h>

constexpr auto RADIAN_TO_DEG_COEFF = 180.0 / M_PI;

void load_params(ros::NodeHandle& nh, MotionConstraints<double>& linear_constraints, MotionConstraints<double>& angular_constraints)
{
    nh.getParam("/hamal/mobile_base_controller/linear/x/max_velocity", linear_constraints.max_increment);
    nh.getParam("/hamal/mobile_base_controller/linear/x/max_acceleration", linear_constraints.acceleration);
    linear_constraints.deacceleration = linear_constraints.acceleration;
    nh.getParam("/hamal/mobile_base_controller/linear/x/max_jerk", linear_constraints.jerk);
    
    nh.getParam("/hamal/mobile_base_controller/angular/z/max_velocity", (angular_constraints.max_increment));
    nh.getParam("/hamal/mobile_base_controller/angular/z/max_acceleration", angular_constraints.acceleration);
    angular_constraints.deacceleration = angular_constraints.acceleration;
    nh.getParam("/hamal/mobile_base_controller/angular/z/max_jerk", angular_constraints.jerk);

    angular_constraints = angular_constraints * RADIAN_TO_DEG_COEFF;
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

int main(int argc, char **argv)
{

    ros::init(argc, argv, "hamal_motion_controller_node");

    MotionProfileController<double, double> linearMotionController;
    MotionProfileController<double, double> angularMotionController;
    MotionConstraints<double> linearMotionConstraints;
    MotionConstraints<double> angularMotionConstraints;

    ros::NodeHandle nh;
    load_params(nh, linearMotionConstraints, angularMotionConstraints);
    linearMotionController.setMotionConstraints(linearMotionConstraints);
    angularMotionController.setMotionConstraints(angularMotionConstraints);
    using namespace hamal_custom_interfaces;
    
    std::queue<MotionProfileCommand> commandQueue;

    diff_drive_controller_hamal::DiffDriveControllerHamalConfig controllerConf;

    ros::Subscriber controllerConfigSub = nh.subscribe<dynamic_reconfigure::Config>(
        "/hamal/mobile_base_controller/parameter_updates",
        1,
        [&](const dynamic_reconfigure::Config::ConstPtr& config){
            auto conf = *config;
            controllerConf.__fromMessage__(conf);

            linearMotionConstraints.max_increment = controllerConf.max_vel_x;
            linearMotionConstraints.acceleration = controllerConf.max_acc_x;
            linearMotionConstraints.deacceleration = controllerConf.min_acc_x;
            linearMotionConstraints.jerk = controllerConf.max_jerk_x;

            angularMotionConstraints.max_increment = (controllerConf.max_vel_z);
            angularMotionConstraints.acceleration = (controllerConf.max_acc_z);
            angularMotionConstraints.deacceleration = (controllerConf.min_acc_z);
            angularMotionConstraints.jerk = (controllerConf.max_jerk_z);
            angularMotionConstraints = angularMotionConstraints * RADIAN_TO_DEG_COEFF;
            linearMotionController.setMotionConstraints(linearMotionConstraints);
            angularMotionController.setMotionConstraints(angularMotionConstraints);

        }
    );
    
    actionlib::SimpleActionServer<hamal_custom_interfaces::MotionProfileOperationAction> controllerSimpleServer(
        nh,
        "motion_profile_controller_server",
        false);

    double controllerFreq = 500.0;

    nh.getParam(
        "/motion_controller/controller_frequency",
        controllerFreq);
    ros::Rate rate(controllerFreq);
    bool goalActive = false;
    const double loopPeriod = 1.0 / controllerFreq;

    std::string velCmdTopicName;
    nh.getParam("/motion_controller/velocity_cmd_topic_name", velCmdTopicName);

    double xTolerance, yawTolerance = 0.0;
    nh.getParam("/motion_controller/x_tolerance", xTolerance);
    nh.getParam("motion_controller/yaw_tolerance", yawTolerance);

    if(velCmdTopicName.empty())
    {
        ROS_ERROR("Velocity command topic name is empty. Shutting down...");
        return 0;
    }

    auto cmdVelPub = nh.advertise<geometry_msgs::Twist>(
        velCmdTopicName,
        1
    );

    nav_msgs::Odometry previousOdom;
    nav_msgs::Odometry currentOdom;

    auto odomSub = nh.subscribe<nav_msgs::Odometry>(
        "/hamal/mobile_base_controller/odom",
        1,
        [&](const nav_msgs::Odometry::ConstPtr& odom_msg)
        {
            currentOdom = *odom_msg;
        }
    );

    controllerSimpleServer.registerGoalCallback(
        [&commandQueue, &controllerSimpleServer]()
        {
            if(controllerSimpleServer.isNewGoalAvailable())
            {
                auto newGoal = controllerSimpleServer.acceptNewGoal()->profile_information;
                commandQueue.push(newGoal);
            }    
            
            
        });

    controllerSimpleServer.registerPreemptCallback(
        [&controllerSimpleServer, &goalActive, &commandQueue]()
        {
            goalActive = false;
            commandQueue.pop();
            controllerSimpleServer.setPreempted();
        });

    double x_target, yaw_target = 0.0;/* std::numeric_limits<double>::quiet_NaN(); */
    geometry_msgs::Twist cmdVel;

    auto resetCmdVel = [&cmdVel, &x_target, &yaw_target]()
    {
        
        //x_target, yaw_target = std::numeric_limits<double>::quiet_NaN();

        cmdVel.angular.x = 0.0;
        cmdVel.angular.y = 0.0;
        cmdVel.angular.z = 0.0;

        cmdVel.linear.x = 0.0;
        cmdVel.linear.y = 0.0;
        cmdVel.linear.z = 0.0;
    };
    
    linearMotionController.setMotionType(MotionProfileType::Trapezoidal);
    angularMotionController.setMotionType(MotionProfileType::Trapezoidal);
    
    controllerSimpleServer.start();
    auto prevUpdateTime = std::chrono::high_resolution_clock::now();

    double realDisplacementSinceProfileStartedX = 0.0;
    double realDisplacementSinceProfileStartedYaw = 0.0;

    auto activeGoalProcedureX = [&](const MotionProfileCommand& active_command, bool& goal_done_flag, const double& elapsed_time) -> bool
    {    
        realDisplacementSinceProfileStartedX += /* std::abs(
            currentOdom.pose.pose.position.x - 
            previousOdom.pose.pose.position.x
        ); */

        std::abs(previousOdom.twist.twist.linear.x * elapsed_time);
        const auto linearRef = linearMotionController.generateMotionProfileReference(active_command.x_target);
        //if(linearRef)
        //{
        //    std::cout << "Target velocity:" << linearRef->velocity << std::endl;
        //}

        if(bool currentInTolerance = inRange(realDisplacementSinceProfileStartedX, std::abs(x_target), xTolerance))
        {
            cmdVel.linear.x = 0.0;
            /* if(!linearRef && !currentInTolerance)
            {
                goal_done_flag = false;
                return false;
            } */

            goal_done_flag = true;
            return true;
        }


        if(linearRef)
        {
            cmdVel.linear.x = linearRef->velocity;
        }
        else
        {
            ROS_ERROR("Error during linear velocity reference generation.");
            cmdVel.linear.x = 0.0;
            return false;
        }
        goal_done_flag = false;
        return true;

    };

    auto activeGoalProcedureYaw = [&](const MotionProfileCommand& active_command, bool& goal_done_flag, const double& elapsed_time) -> bool
    {
        
        const auto angularRef = angularMotionController.generateMotionProfileReference(active_command.yaw_target);
        if(!angularRef)
        {   
            goal_done_flag = true;
            return true;
        }
        realDisplacementSinceProfileStartedYaw += /* std::abs(
            quaternionToDegree(currentOdom.pose.pose.orientation) -
            quaternionToDegree(previousOdom.pose.pose.orientation)
        ); */
        
        std::abs((previousOdom.twist.twist.angular.z * RADIAN_TO_DEG_COEFF) * elapsed_time);    
/*         std::cout << "current angular vel: " << previousOdom.twist.twist.angular.z << std::endl;
 */        
        /* std::cout << ang */
        if(bool currentInTolerance = inRange(realDisplacementSinceProfileStartedYaw, std::abs(yaw_target), yawTolerance))
        {
            std::cout << "Yaw in tolerance\n";
            cmdVel.angular.z = 0.0;
            /* if(!angularRef && !currentInTolerance)
            {
                goal_done_flag = false;
                return false;cd
            } */
            goal_done_flag = true;
            return true;
        }

/*         const auto angularRef = angularMotionController.generateMotionProfileReference(active_command.yaw_target);
 */
        if(angularRef)
        {
            cmdVel.angular.z = angularRef->velocity * (1.0 / RADIAN_TO_DEG_COEFF);
            std::cout << angularRef->velocity << " " << cmdVel.angular.z << std::endl;
        }
        else
        {
            ROS_ERROR("Error during angular velocity reference generation.");
            cmdVel.angular.z = 0.0;
            return false;
        }
        goal_done_flag = false;
        return true;
    };

    auto currentTime = ros::Time::now();
    ros::Time previosTime = currentTime;

    while (ros::ok())
    {   
        ros::spinOnce();
        currentTime = ros::Time::now();
        double loopUpdateElapsedTime = (currentTime - previosTime).toSec();
        previosTime = currentTime;
        if (commandQueue.empty() && !goalActive)
        {
            //ROS_INFO("No commands arrived. Waiting...");
            rate.sleep();
            realDisplacementSinceProfileStartedX = 0.0;
            realDisplacementSinceProfileStartedYaw = 0.0;
            continue;
        }

        const auto& activeCommand = commandQueue.front();
        bool goalActiveInX = activeCommand.x_target != 0 ? true : false;
        bool goalActiveInYaw = activeCommand.yaw_target != 0 ? true : false;

        // Setup profile generation.
        if (!goalActive && !commandQueue.empty())
        {
            linearMotionController.reset();
            angularMotionController.reset();
            linearMotionController.setMotionConstraints(linearMotionConstraints);
            angularMotionController.setMotionConstraints(angularMotionConstraints);
            const auto newGoal = commandQueue.front();
            if(newGoal.profile_type != 0)
            {
                linearMotionController.setMotionType(static_cast<MotionProfileType>(newGoal.profile_type - 1));
                angularMotionController.setMotionType(static_cast<MotionProfileType>(newGoal.profile_type - 1));
            }

            realDisplacementSinceProfileStartedX = 0.0;
            realDisplacementSinceProfileStartedYaw = 0.0;
            /* currentOdom = nav_msgs::Odometry();
            previousOdom = currentOdom; */
            
            if (newGoal.x_target != 0.0)
            {

                bool setupOk = linearMotionController.setupProfile(
                    newGoal.x_target/* ,
                    static_cast<MotionProfileType>(newGoal.profile_type + 1) */);
                if(!setupOk)
                {
                    commandQueue.pop();
                    goalActive = false;
                    controllerSimpleServer.setAborted();
                    ROS_ERROR("Can not setup profile with the given parameters.");
                    continue;
                }

                /* initialProfileDurationX = linearMotionController */
                previousOdom = currentOdom;
                x_target = newGoal.x_target;
            }

            if (newGoal.yaw_target != 0.0)
            {
                bool setupOk = angularMotionController.setupProfile(
                    newGoal.yaw_target/* ,
                    static_cast<MotionProfileType>(newGoal.profile_type + 1) */);
                if(!setupOk)
                {
                    commandQueue.pop();
                    goalActive = false;
                    controllerSimpleServer.setAborted();
                    ROS_ERROR("Can not setup profile with the given parameters.");
                    continue;
                }
                previousOdom = currentOdom;
                yaw_target = newGoal.yaw_target;
            }          

            if(newGoal.x_target != 0.0 && newGoal.yaw_target != 0.0)
            {
                
                double initialProfileDurationX, initialProfileDurationYaw = 0.0;

                initialProfileDurationX = linearMotionController.m_CurrentProfileInformation.m_CurrentProfileTimes->getTotalTime();
                initialProfileDurationYaw = angularMotionController.m_CurrentProfileInformation.m_CurrentProfileTimes->getTotalTime();

                if(initialProfileDurationX != initialProfileDurationYaw)
                {
                
                    if(initialProfileDurationX > initialProfileDurationYaw)
                    {
                        // X movement is longer than yaw movement
                        // Slow down on the yaw axis
                        const auto ratio =  initialProfileDurationX / initialProfileDurationYaw;
                        const auto tempAngularMotionConstraints = angularMotionConstraints * ratio;
                        angularMotionController.setMotionConstraints(tempAngularMotionConstraints);
                        bool setupOk = angularMotionController.setupProfile(
                        newGoal.yaw_target);
                        if(!setupOk)
                        {
                            commandQueue.pop();
                            goalActive = false;
                            controllerSimpleServer.setAborted();
                            ROS_ERROR("Can not setup profile with the given parameters.");
                            continue;
                        }
                    }
                    else
                    {   
                        // Yaw movement is longer than X movement
                        // Slow down on the x-Axis:
                        const auto ratio = initialProfileDurationYaw / initialProfileDurationX;
                        const auto tempLinearMotionConstraints = linearMotionConstraints * ratio;
                        linearMotionController.setMotionConstraints(tempLinearMotionConstraints);
                        bool setupOk = linearMotionController.setupProfile(
                        newGoal.x_target/* ,
                        static_cast<MotionProfileType>(newGoal.profile_type + 1) */);
                        if(!setupOk)
                        {
                            commandQueue.pop();
                            goalActive = false;
                            controllerSimpleServer.setAborted();
                            ROS_ERROR("Can not setup profile with the given parameters.");
                            continue;
                        }
                    }
                }

            }
            
            goalActive = true;
            
        }

        if (goalActive)
        {   
            
            bool goalOkX, goalOkYaw = false;
            bool goalDoneX, goalDoneYaw = false;
            bool currentGoalSuccessful = false;
            if(goalActiveInX) 
            {
                goalOkX = activeGoalProcedureX(activeCommand, goalDoneX, loopUpdateElapsedTime);

                if(goalDoneX)
                {
                    std::cout << "X goal done." << std::endl;
                    currentGoalSuccessful = true;   
                }
                else
                {
                    currentGoalSuccessful = false;
                }
            }
            else{goalOkX = true;}

            if(goalActiveInYaw)
            {
                goalOkYaw = activeGoalProcedureYaw(activeCommand, goalDoneYaw, loopUpdateElapsedTime);
                
                if(goalDoneYaw)
                {
                    std::cout << "Yaw goal done." << std::endl;
                    if(!(goalActiveInX && !goalDoneX))
                        currentGoalSuccessful = true;
                }
                else
                {
                    currentGoalSuccessful = false;
                }
            }
            else{goalOkYaw = true;} 

            if(currentGoalSuccessful)
            {
                std::cout << "Goal is done successfuly.\n";
                std::cout << "Delta Dx: " << realDisplacementSinceProfileStartedX << std::endl;
                std::cout << "Delta Dyaw: " << realDisplacementSinceProfileStartedYaw << std::endl;

                
                hamal_custom_interfaces::MotionProfileOperationResult res;
                res.target_reached = true;
                controllerSimpleServer.setSucceeded(res);
                commandQueue.pop();
                goalActive = false;
            }
            
            if(!(goalOkYaw && goalOkX))
            {
                /* ROS_ERROR("Error during reference generation."); */
                std::cout << "Delta Dx: " << realDisplacementSinceProfileStartedX << std::endl;
                std::cout << "Delta Dyaw: " << realDisplacementSinceProfileStartedYaw << std::endl;
                hamal_custom_interfaces::MotionProfileOperationResult res;
                res.target_reached = false;
                controllerSimpleServer.setAborted(res);
                commandQueue.pop();
                goalActive = false;
            }

            previousOdom = currentOdom;
            cmdVelPub.publish(cmdVel);
            resetCmdVel();        
        }

        rate.sleep();
    }

    return 0;
}