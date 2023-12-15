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

    if(da >= abs(control_var) || dd >= abs(control_var)){
        ROS_INFO("Creating triangular profile instead.");
        max_vel = std::sqrt(std::abs(control_var) * max_accel);
        /* double totalTimeFromMaxVel = (2.0 * abs(control_var)) / max_vel;
        double totalTimeFromMaxAcc = (2.0 * sqrt(abs(control_var))) / sqrt(max_accel);
        double maxTotalTime = totalTimeFromMaxVel;
        if(totalTimeFromMaxAcc != totalTimeFromMaxVel){
            maxTotalTime = std::max(totalTimeFromMaxVel, totalTimeFromMaxAcc);
            ROS_INFO("Maximum of two times: %f", maxTotalTime);
        } */
        
        
        ta = max_vel / max_accel;
        td = ta;

        da = 0.5 * ta * max_vel;
        dd = da;

        ROS_INFO("Triangular profile times: Ta: %f | Td :%f", ta, td);
        ROS_INFO("Distances: Ta: %f | Td: %f", da, dd);

    } 

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
    using namespace hamal_custom_interfaces;

    auto goalActionServer = actionlib::ActionServer<MotionProfileOperationAction>(
        nh,
        "trapezoidal_motion_profile_server",
        false
    );
    actionlib::ServerGoalHandle<hamal_custom_interfaces::MotionProfileOperationAction> currentGoalCopy;
    bool cancelGoal = false;
    goalActionServer.registerGoalCallback(
        [&](actionlib::ServerGoalHandle<hamal_custom_interfaces::MotionProfileOperationAction> goal_handle) -> void{
            
            currentGoalCopy = goal_handle;
            auto goalInfo = currentGoalCopy.getGoal()->profile_information;

            if(goalInfo.x_target != 0.0 && goalInfo.yaw_target != 0.0){
                hamal_custom_interfaces::ProfileCommand cmdLinear;
                hamal_custom_interfaces::ProfileCommand cmdAngular;

                cmdAngular.linear = false;
                cmdAngular.target = goalInfo.yaw_target;
                cmdAngular.max_vel = goalInfo.max_velocity_yaw;
                cmdAngular.max_acc = goalInfo.max_accelaration_x;

                commandQueue.push(cmdAngular);

                cmdLinear.linear = true;
                cmdLinear.target = goalInfo.x_target;
                cmdLinear.max_vel = goalInfo.max_velocity_x;
                cmdLinear.max_acc = goalInfo.max_accelaration_x;
                
                commandQueue.push(cmdLinear);

            }
            else if(goalInfo.x_target != 0.0){

                hamal_custom_interfaces::ProfileCommand cmdLinear;
                cmdLinear.linear = true;
                cmdLinear.target = goalInfo.x_target;
                cmdLinear.max_vel = goalInfo.max_velocity_x;
                cmdLinear.max_acc = goalInfo.max_accelaration_x;
                
                commandQueue.push(cmdLinear);
            }
            else if(goalInfo.yaw_target != 0.0){
                hamal_custom_interfaces::ProfileCommand cmdAngular;

                cmdAngular.linear = false;
                cmdAngular.target = goalInfo.yaw_target;
                cmdAngular.max_vel = goalInfo.max_velocity_yaw;
                cmdAngular.max_acc = goalInfo.max_accelaration_x;

                commandQueue.push(cmdAngular);
            }
            else{
                hamal_custom_interfaces::MotionProfileOperationResult res;
                res.target_reached = false;
                currentGoalCopy.setRejected(res, "Both targets are zero, rejecting goal...");
            }

            hamal_custom_interfaces::MotionProfileOperationResult res;
            res.target_reached = true;
            currentGoalCopy.setAccepted();
            cancelGoal = false;
            
        }
    );

    goalActionServer.registerCancelCallback([&](actionlib::ServerGoalHandle<hamal_custom_interfaces::MotionProfileOperationAction> goal_handle)->void{
            cancelGoal = true;
    });

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

    ros::Publisher odomValuePub = nh.advertise<std_msgs::Float64>(
        "/vel_value",
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
    PID linearPositionController;
    kp, ki, kd = 0.0;
    nh.param(
        "/profile_controller_params/trapezoidal_profile/linear_position_controller/kp",
        kp,
        kp
    );

    nh.param(
        "/profile_controller_params/trapezoidal_profile/linear_position_controller/ki",
        ki,
        ki
    );

    nh.param(
        "/profile_controller_params/trapezoidal_profile/linear_position_controller/kd",
        kd,
        kd
    );
    linearPositionController.setParams(kp, ki, kd);

    PID angularPositionController;
    kp, ki, kd = 0.0;
    nh.param(
        "/profile_controller_params/trapezoidal_profile/angular_position_controller/kp",
        kp,
        kp
    );

    nh.param(
        "/profile_controller_params/trapezoidal_profile/angular_position_controller/ki",
        ki,
        ki
    );

    nh.param(
        "/profile_controller_params/trapezoidal_profile/angular_position_controller/kd",
        kd,
        kd
    );
    angularPositionController.setParams(kp, ki, kd);

    /* boost::recursive_mutex reconfMutex;

    std::shared_ptr<dynamic_reconfigure::Server<hamal_control::MotionControllerConfig>> dynamicReconfigServer = std::make_shared<dynamic_reconfigure::Server<hamal_control::MotionControllerConfig>>(
        reconfMutex,
        nh
    ); */

    /* dynamic_reconfigure::Server<hamal_control::MotionControllerConfig>::CallbackType cb;
    cb = boost::bind(&motion_profile_dynamic_reconfigure_callback, _1, _2, velController, linearPositionController, angularPositionController); */
    
    hamal_control::MotionControllerConfig conf;
    conf.velocity_controller_kp = velController.getCurrentParams()[0];
    conf.velocity_controller_ki = velController.getCurrentParams()[1];
    conf.velocity_controller_kd = velController.getCurrentParams()[2];

    conf.linear_position_controller_kp = linearPositionController.getCurrentParams()[0];
    conf.linear_position_controller_ki = linearPositionController.getCurrentParams()[1];
    conf.linear_position_controller_kd = linearPositionController.getCurrentParams()[2];

    conf.angular_position_controller_kp = angularPositionController.getCurrentParams()[0];
    conf.angular_position_controller_ki = angularPositionController.getCurrentParams()[1];
    conf.angular_position_controller_kd = angularPositionController.getCurrentParams()[2];
    

    /* reconfMutex.lock();
    dynamicReconfigServer->updateConfig(conf);
    reconfMutex.unlock(); */
    /* dynamicReconfigServer->setCallback(
        cb
    ); */

    
    double orientationSetPoint = 0.0;
    bool controlOrientation = true; 
    double distanceWithoutFeedback = 0.0;
    double distanceWithControlledVelReference = 0.0;
    double realDistanceTraveled = 0.0;
    double prevX = currentOdom.pose.pose.position.x;
    double posControlledDistance = 0.0;

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
        posControlledDistance = 0.0;
    };

    while(ros::ok())
    {
        ros::spinOnce();

        if(cancelGoal){

            resetVars();
        }
        
        std_msgs::Float64 vel;
            /* vel.data = currentOdom.twist.twist.linear.x; */
            vel.data = currentOdom.twist.twist.linear.x;

            odomValuePub.publish(vel);
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
                    ROS_WARN("Can not get orientation.");
                    controlOrientation = false;
                }

                if(currentTfOpt){
                    auto orientationQuat = currentTfOpt.value().transform.rotation;
                    orientationSetPoint = quaternionToDegree(orientationQuat);
                    controlOrientation = true;
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
                if(inRange(0.05, abs(goalInfo.target_position), (goalInfo.motion_type == MotionType::Linear ? abs(realDistanceTraveled) : abs(realDistanceTraveledAngular)))){
                    hamal_custom_interfaces::MotionProfileOperationResult res;
                    res.target_reached = true;
                    currentGoalCopy.setSucceeded(res);    
                }
                else{
                    hamal_custom_interfaces::MotionProfileOperationResult res;
                    res.target_reached = false;
                    currentGoalCopy.setAborted(res);
                }
                continue;
            }
            else if(inRange(0.001, abs(goalInfo.target_position), (goalInfo.motion_type == MotionType::Linear ? abs(realDistanceTraveled) : abs(realDistanceTraveledAngular)))){
                currentGoalCopy.setSucceeded();
                resetVars();
                hamal_custom_interfaces::MotionProfileOperationResult res;
                res.target_reached = true;
                currentGoalCopy.setSucceeded(res);
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
                        ROS_WARN("Can not get TF.");
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
            int dir = 1.0;
            if(goalInfo.target_position < 0){
                    dir = -1.0;
            }
            if(timeSinceProfileStart <= goalInfo.ta){
                velRef = goalInfo.uncontrolled_vel_ref + (elaspedLoopTime * goalInfo.max_acc * dir) ;
            }
            else if(((timeSinceProfileStart > goalInfo.ta) && timeSinceProfileStart <= (goalInfo.ta + goalInfo.tc)) && goalInfo.tc > 0.0){
                velRef = goalInfo.max_vel * dir;
            }
            else if((timeSinceProfileStart >= goalInfo.ta + goalInfo.tc) && (timeSinceProfileStart <= goalInfo.ta + goalInfo.tc + goalInfo.td)){
                velRef = goalInfo.uncontrolled_vel_ref - (elaspedLoopTime * goalInfo.max_acc * dir);
            }
            goalInfo.uncontrolled_vel_ref = velRef;
            /* if(orientTfOpt && controlOrientation){
                const auto orientTf = orientTfOpt.value();
                double orientCtrl = linearMoveOrientationController.control(orientationSetPoint, quaternionToDegree(orientTf.transform.rotation), currTime);
                goalInfo.angular_vel_ref = orientCtrl * elaspedLoopTime;
                double err = quaternionToDegree(orientTf.transform.rotation) - orientationSetPoint;
                ROS_INFO("Orientation error: %f | Angular velocity to control: %f", err, goalInfo.angular_vel_ref);
            }else{ROS_WARN("Not controlling orientation.");} */
            
            double posControlOutput = 0.0;
            if(goalInfo.motion_type == MotionType::Linear){
                posControlOutput = linearPositionController.control(distanceWithoutFeedback, realDistanceTraveled, currTime);
            }
            else if(goalInfo.motion_type == MotionType::Angular){
                posControlOutput = angularPositionController.control(distanceWithoutFeedback, realDistanceTraveledAngular, currTime);
            }
            
            double velControlOutput = velController.control(currentValue, goalInfo.vel_ref, currTime);
            /* double velControlOutput = 0.0; */
            distanceWithoutFeedback +=  velRef * elaspedLoopTime;
            goalInfo.vel_ref = velRef + posControlOutput + velControlOutput;
            /* auto controlledRef = velController.control(currentValue, goalInfo.vel_ref, currTime); */
            /* controlledRef += velRef; */
            
            /* distanceWithoutFeedback += controlledRef * elaspedLoopTime; */

            ROS_INFO("Distance traveled (feedback): %f | Distance traveled (open-loop): %f", realDistanceTraveled, distanceWithoutFeedback);
            /* if(goalInfo.target_position < 0){
                velRef = velRef * -1.0;
            } */

            /* goalInfo.vel_ref = velRef; */
            geometry_msgs::Twist cmdVel;
            /* double positionControlledRef = velRef + posControlOutput; */
            /* posControlledDistance += positionControlledRef * elaspedLoopTime; */            

            if(goalInfo.motion_type == MotionType::Angular){
                cmdVel.angular.z = goalInfo.vel_ref;
            }
            else{
                cmdVel.linear.x = goalInfo.vel_ref;
            }
            distanceWithControlledVelReference += goalInfo.vel_ref * elaspedLoopTime;

            /* ROS_INFO("Distance traveled via position controlled reference: %f", posControlledDistance);
            ROS_INFO("Distance traveled (controlled): %f", distanceWithControlledVelReference); */
            
            ROS_INFO("Velocity Reference: %f | Controlled reference: %f", velRef, goalInfo.vel_ref);
            if(cmdVelRtPub->trylock()){
                    cmdVelRtPub->msg_ = cmdVel;
                    cmdVelRtPub->unlockAndPublish();
            }

            geometry_msgs::TwistStamped controllerCmdVel;
            controllerCmdVel.header.stamp = ros::Time::now();
            controllerCmdVel.twist.linear.x = goalInfo.vel_ref;
            controlledCmdVelPub.publish(controllerCmdVel);
            
        }


        rate.sleep();
    }

    return 0;
}
