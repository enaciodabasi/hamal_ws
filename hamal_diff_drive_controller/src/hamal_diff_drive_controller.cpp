/**
 * @file hamal_diff_drive_controller.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "hamal_diff_drive_controller/hamal_diff_drive_controller.hpp"

namespace hamal_diff_drive_controller
{
    HamalDiffDriveController::HamalDiffDriveController()
    {

    }

    bool HamalDiffDriveController::init(
        hardware_interface::VelocityJointInterface* hw,
        ros::NodeHandle& base_nh, 
        ros::NodeHandle& controller_nh)
    {   
        const std::string complete_ns = controller_nh.getNamespace();
        std::size_t id = complete_ns.find_last_of("/");
        m_ControllerName = complete_ns.substr(id + 1);

        std::string leftWheelJointName;
        if(controller_nh.hasParam("left_wheel"))
        {
            controller_nh.getParam("left_wheel", leftWheelJointName);
        }
        else
        {

            return false;
        }
        m_LeftWheel.jointHandle = hw->getHandle(leftWheelJointName);
        
        std::string rightWheelJointName;
        if(controller_nh.hasParam("right_wheel"))
        {
            controller_nh.getParam("right_wheel", rightWheelJointName);
        }
        else
        {

            return false;
        }

        m_RightWheel.jointHandle = hw->getHandle(rightWheelJointName);

        controller_nh.param("publish_rate", m_PublishRate, 50.0);
        m_PublishPeriod = ros::Duration(m_PublishRate);

        int odomRollingWindowSize;
        controller_nh.param("velocity_rolling_window_size", odomRollingWindowSize, 10);


        if(controller_nh.hasParam("wheel_radius"))
        {
            controller_nh.getParam("wheel_radius", m_WheelParams.wheel_radius);
        }
        else
        {
            return false;
        }

        if(controller_nh.hasParam("wheel_separation"))
        {
            controller_nh.getParam("wheel_separation", m_WheelParams.wheel_separation);
        }
        else 
        {
            return false;
        }

        if(controller_nh.hasParam("use_position_for_odom"))
        {
            controller_nh.getParam("use_position_for_odom", m_UsePositionForOdom);
        }
        if(controller_nh.hasParam("use_velocity_for_odom"))
        {
            controller_nh.getParam("use_velocity_for_odom", m_UseVelocityForOdom);
        }
        
        // Create Odometry Objects:
        if(m_UsePositionForOdom)
            m_PosOdom = std::make_unique<PositionOdometry>(
                m_WheelParams, (std::size_t)odomRollingWindowSize
            );

        if(m_UseVelocityForOdom)
            m_VelOdom = std::make_unique<VelocityOdometry>(
                m_WheelParams, (std::size_t)odomRollingWindowSize
            );

        if(m_UsePositionForOdom && m_UseVelocityForOdom)
        {
            m_UseBothOdom = true;
        }
        else
        {
            m_UseBothOdom = false;
        }

        if(controller_nh.hasParam("odom_tf_enabled"))
        {
            controller_nh.getParam("odom_tf_enabled", m_OdomTfEnabled);
        }

        /* if(controller_nh.hasParam("publish_unfiltered_odom"))
        {
            controller_nh.getParam("publish_unfiltered_odom", m_PublishUnfilteredOdom);
        } */
        
        controller_nh.param("base_frame_id", m_BaseFrameID, m_BaseFrameID);
        controller_nh.param("odom_frame_id", m_OdomFrameID, m_OdomFrameID);

        std::vector<double> poseCovariance;
        poseCovariance.reserve(6);
        
        std::vector<double> twistCovariance;
        twistCovariance.reserve(6);
        
        controller_nh.param("pose_covariance", poseCovariance, {0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.03});
        controller_nh.param("twist_covariance", twistCovariance, {0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.03});

        nav_msgs::Odometry templateOdomMsg;
        templateOdomMsg.header.frame_id = m_OdomFrameID;
        templateOdomMsg.child_frame_id = m_BaseFrameID;
        templateOdomMsg.pose.pose.position.z = 0.0;
        templateOdomMsg.twist.twist.linear.y = 0.0;
        templateOdomMsg.twist.twist.linear.z = 0.0;
        templateOdomMsg.twist.twist.angular.x = 0.0;
        templateOdomMsg.twist.twist.angular.y = 0.0;
        templateOdomMsg.pose.covariance = {
            static_cast<double>(poseCovariance[0]), 0., 0., 0., 0., 0.,
            0., static_cast<double>(poseCovariance[1]), 0., 0., 0., 0.,
            0., 0., static_cast<double>(poseCovariance[2]), 0., 0., 0.,
            0., 0., 0., static_cast<double>(poseCovariance[3]), 0., 0.,
            0., 0., 0., 0., static_cast<double>(poseCovariance[4]), 0.,
            0., 0., 0., 0., 0., static_cast<double>(poseCovariance[5])
        };
        templateOdomMsg.twist.covariance = {
            static_cast<double>(twistCovariance[0]), 0., 0., 0., 0., 0.,
            0., static_cast<double>(twistCovariance[1]), 0., 0., 0., 0.,
            0., 0., static_cast<double>(twistCovariance[2]), 0., 0., 0.,
            0., 0., 0., static_cast<double>(twistCovariance[3]), 0., 0.,
            0., 0., 0., 0., static_cast<double>(twistCovariance[4]), 0.,
            0., 0., 0., 0., 0., static_cast<double>(twistCovariance[5])
        };

        if(m_UseBothOdom)
        {
            /* m_FilteredOdomPublisher = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::Odometry>>(
                controller_nh, 
                m_FilteredOdomTopicName, 
                100
            );

            m_FilteredOdomPublisher->msg_ = templateOdomMsg; */
            m_UnfilteredPositionOdomPub = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::Odometry>>(
                    controller_nh,
                    m_UnfilteredPosOdomTopicName,
                    100
                );

            m_UnfilteredPositionOdomPub->msg_ = templateOdomMsg;

            m_UnfilteredVelocityOdomPub = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::Odometry>>(
                controller_nh,
                m_UnfilteredVelOdomTopicName,
                100
            );

            m_UnfilteredVelocityOdomPub->msg_ = templateOdomMsg;

        }
        else if(m_UsePositionForOdom && !m_UseVelocityForOdom)
        {
            m_UnfilteredPositionOdomPub = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::Odometry>>(
                controller_nh,
                m_UnfilteredPosOdomTopicName,
                100
            );

            m_UnfilteredPositionOdomPub->msg_ = templateOdomMsg;

        }
        else if(m_UseVelocityForOdom && !m_UsePositionForOdom)
        {
            m_UnfilteredVelocityOdomPub = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::Odometry>>(
                controller_nh,
                m_UnfilteredVelOdomTopicName,
                100
            );

            m_UnfilteredVelocityOdomPub->msg_ = templateOdomMsg;
        }

        m_VelCommandTimeout =  controller_nh.param("twist_command_timeout", m_VelCommandTimeout, m_VelCommandTimeout); 

        if(m_OdomTfEnabled) // TF is independent from the type of odometry calculated
        {
            m_OdomTfPublisher = std::make_shared<realtime_tools::RealtimePublisher<tf::tfMessage>>(
                base_nh, 
                "/tf", 
                100
            );

            m_OdomTfPublisher->msg_.transforms.resize(1);
            m_OdomTfPublisher->msg_.transforms.at(0).header.frame_id = m_OdomFrameID;
            m_OdomTfPublisher->msg_.transforms.at(0).child_frame_id = m_BaseFrameID;
            m_OdomTfPublisher->msg_.transforms.at(0).transform.translation.z = 0.0;
        }

        controller_nh.param("twist_cmd_topic_name", m_TwistCmdTopicName, m_TwistCmdTopicName);

        m_TwistCmdSub = controller_nh.subscribe(
            m_TwistCmdTopicName,
            1,
            &HamalDiffDriveController::twistCmdCallback,
            this
        );

        // Initialize limiters
        bool hasLinearVelLimit = true;
        bool hasLinearAccelLimit = true;
        bool hasLinearJerkLimit = false;
        controller_nh.param("limiter/linear_x/has_velocity_limits", hasLinearVelLimit, hasLinearVelLimit);
        controller_nh.param("limiter/linear_x/has_accel_limits", hasLinearAccelLimit, hasLinearAccelLimit);
        controller_nh.param("limiter/linear_x/has_jerk_limits", hasLinearJerkLimit, hasLinearJerkLimit);
        
        bool hasAngularVelLimit = true;
        bool hasAngularAccelLimit = true;
        bool hasAngularJerkLimit = false;
        controller_nh.param("limiter/angular_z/has_velocity_limits", hasAngularVelLimit, hasAngularVelLimit);
        controller_nh.param("limiter/angular_z/has_accel_limits", hasAngularAccelLimit, hasAngularAccelLimit);
        controller_nh.param("limiter/angular_z/has_jerk_limits", hasAngularJerkLimit, hasAngularJerkLimit);

        m_LinearVelLimiter = std::make_unique<Limiter>(hasLinearVelLimit, hasLinearAccelLimit, hasLinearJerkLimit);
        m_AngularVelLimiter = std::make_unique<Limiter>(hasAngularVelLimit, hasAngularAccelLimit, hasAngularJerkLimit);
        
        double lin_max_vel, lin_min_vel = 0.0;
        controller_nh.param("limiter/linear_x/max_vel", lin_max_vel, lin_max_vel);
        controller_nh.param("limiter/linear_x/min_vel", lin_min_vel, lin_min_vel);
        m_LinearVelLimiter->setVelLimits(lin_min_vel, lin_max_vel);
        
        double lin_max_accel, lin_min_accel = 0.0;
        controller_nh.param("limiter/linear_x/max_accel", lin_max_accel, lin_max_accel);
        controller_nh.param("limiter/linear_x/min_accel", lin_min_accel, lin_min_accel);
        m_LinearVelLimiter->setAccelLimits(lin_min_accel, lin_max_accel);

        double lin_max_jerk, lin_min_jerk = 0.0;
        controller_nh.param("limiter/linear_x/max_jerk", lin_max_jerk, lin_max_jerk);
        controller_nh.param("limiter/linear_x/min_jerk", lin_min_jerk, lin_min_jerk);
        m_LinearVelLimiter->setJerkLimits(lin_min_jerk, lin_max_jerk);

        double ang_max_vel, ang_min_vel = 0.0;
        controller_nh.param("limiter/angular_z/max_vel", ang_max_vel, ang_max_vel);
        controller_nh.param("limiter/angular_z/min_vel", ang_min_vel, ang_min_vel);
        m_AngularVelLimiter->setVelLimits(ang_min_vel, ang_max_vel);
        
        double ang_max_accel, ang_min_accel = 0.0;
        controller_nh.param("limiter/angular_z/max_accel", ang_max_accel, ang_max_accel);
        controller_nh.param("limiter/angular_z/min_accel", ang_min_accel, ang_min_accel);
        m_AngularVelLimiter->setAccelLimits(ang_min_accel, ang_max_accel);

        double ang_max_jerk, ang_min_jerk = 0.0;
        controller_nh.param("limiter/angular_z/max_jerk", ang_max_jerk, ang_max_jerk);
        controller_nh.param("limiter/angular_z/min_jerk", ang_min_jerk, ang_min_jerk);
        m_AngularVelLimiter->setJerkLimits(ang_min_jerk, ang_max_jerk);
        
        // Dynamic Parameters:
        HamalDiffDriveControllerConfig conf;

        m_DynamicParamServer = std::make_shared<dynamic_reconfigure::Server<HamalDiffDriveControllerConfig>>(m_DynamicParamMutex, controller_nh);
        m_DynamicParamMutex.lock();
        m_DynamicParamServer->updateConfig(conf);
        m_DynamicParamMutex.unlock();

        m_DynamicParamServer->setCallback(std::bind(
            &HamalDiffDriveController::dynamicReconfigureCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        ));

        return true;

    }

    void HamalDiffDriveController::starting(const ros::Time& time)
    {
        if(m_UsePositionForOdom)
            m_PosOdom->init(time);

        if(m_UseVelocityForOdom)
            m_VelOdom->init(time);

        m_PreviousTime = time;
        m_OdomControlTime = time;
    }

    void HamalDiffDriveController::update(const ros::Time& time, const ros::Duration& period)
    {
        // Update dynamic parameters:

        auto newParamsPtr = m_DynamicParamsBuffer.readFromRT();
        if(newParamsPtr)
        {
            
            const auto newParams = *newParamsPtr;

            m_LinearVelLimiter->setVelLimits(newParams.linearVelLimit);
            m_LinearVelLimiter->setAccelLimits(newParams.linearAccelLimit);
            m_LinearVelLimiter->setJerkLimits(newParams.linearJerkLimit);

            m_AngularVelLimiter->setVelLimits(newParams.angularVelLimit);
            m_AngularVelLimiter->setAccelLimits(newParams.angularAccelLimit);
            m_AngularVelLimiter->setJerkLimits(newParams.angularJerkLimit);
            
            delete newParamsPtr;
        }

        if(m_UsePositionForOdom)
        {
            const auto leftPos = m_LeftWheel.jointHandle.getPosition();
            const auto rightPos = m_RightWheel.jointHandle.getPosition();

            m_PosOdom->update(leftPos, rightPos, time);
        }

        if(m_UseVelocityForOdom)
        {
            const auto leftVel = m_LeftWheel.jointHandle.getVelocity();
            const auto rightVel = m_RightWheel.jointHandle.getVelocity();

            m_VelOdom->update(leftVel, rightVel, time);
        }


        if(m_OdomControlTime + m_PublishPeriod < time)
        {

            if(m_UsePositionForOdom && m_UnfilteredPositionOdomPub->trylock())
            {
                const geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(m_PosOdom->getPose().heading);
                m_UnfilteredPositionOdomPub->msg_.pose.pose.position.x = m_PosOdom->getPose().x;
                m_UnfilteredPositionOdomPub->msg_.pose.pose.position.y = m_PosOdom->getPose().y;
                m_UnfilteredPositionOdomPub->msg_.pose.pose.orientation = orientation;
                m_UnfilteredPositionOdomPub->msg_.twist.twist.linear.x = m_PosOdom->getVelocity().linear;
                m_UnfilteredPositionOdomPub->msg_.twist.twist.linear.z = m_PosOdom->getVelocity().angular;
                m_UnfilteredPositionOdomPub->unlockAndPublish();
            }

            if(m_UseVelocityForOdom && m_UnfilteredVelocityOdomPub->trylock())
            {
                const geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(m_VelOdom->getPose().heading);
                m_UnfilteredVelocityOdomPub->msg_.pose.pose.position.x = m_VelOdom->getPose().x;
                m_UnfilteredVelocityOdomPub->msg_.pose.pose.position.y = m_VelOdom->getPose().y;
                m_UnfilteredVelocityOdomPub->msg_.pose.pose.orientation = orientation;
                m_UnfilteredVelocityOdomPub->msg_.twist.twist.linear.x = m_VelOdom->getVelocity().linear;
                m_UnfilteredVelocityOdomPub->msg_.twist.twist.linear.z = m_VelOdom->getVelocity().angular;
                m_UnfilteredVelocityOdomPub->unlockAndPublish();
            }

            m_OdomControlTime += m_PublishPeriod;
        }

        // Write Commands:

        VelCommand newVelCommand = *(m_VelCommandBuffer.readFromRT());
        const auto elapsedTime = (time - newVelCommand.timestamp).toSec();

        if(elapsedTime > m_VelCommandTimeout)
        {
            newVelCommand.linear = 0.0;
            newVelCommand.angular = 0.0;
        }

        const double commandDt = period.toSec();
        m_LinearVelLimiter->limit(newVelCommand.linear, m_LastTwoVelCmds.first.linear, m_LastTwoVelCmds.second.linear, commandDt);
        m_AngularVelLimiter->limit(newVelCommand.angular, m_LastTwoVelCmds.first.angular, m_LastTwoVelCmds.second.angular, commandDt);

        m_LastTwoVelCmds.second = m_LastTwoVelCmds.first;
        m_LastTwoVelCmds.first = newVelCommand;

        const double leftWheelVel = (newVelCommand.linear - newVelCommand.angular * m_WheelParams.wheel_separation / 2.0) / m_WheelParams.wheel_radius;
        const double rightWheelVel = (newVelCommand.linear + newVelCommand.angular * m_WheelParams.wheel_separation / 2.0) / m_WheelParams.wheel_radius;
        
        m_LeftWheel.jointHandle.setCommand(leftWheelVel);
        m_RightWheel.jointHandle.setCommand(rightWheelVel);

        m_PreviousTime = time;
    }

    void HamalDiffDriveController::stopping(const ros::Time& time)
    {
        m_LeftWheel.jointHandle.setCommand(0.0);
        m_RightWheel.jointHandle.setCommand(0.0);
    }

    void HamalDiffDriveController::twistCmdCallback(const geometry_msgs::Twist& twist_cmd)
    {
        if(!isRunning())
        {
            return;
        }

        const auto twistCmd = twist_cmd;

        if(std::isnan(twistCmd.angular.z) || std::isnan(twistCmd.linear.x))
        {
            ROS_WARN("NaN value in velocity command.");
            return;
        }

        VelCommand cmd;
        cmd.timestamp = ros::Time::now();
        cmd.linear = twistCmd.linear.x;
        cmd.angular = twistCmd.angular.z;

        m_VelCommandBuffer.writeFromNonRT(
            cmd
        );

    }

    void HamalDiffDriveController::dynamicReconfigureCallback(const HamalDiffDriveControllerConfig& config, uint32_t)
    {
        DynamicParams newParams;
        newParams.linearVelLimit.min = std::move(config.min_vel_x);
        newParams.linearVelLimit.max = std::move(config.max_vel_x);
        
        newParams.angularVelLimit.min = std::move(config.min_vel_z);
        newParams.angularVelLimit.max = std::move(config.max_vel_z);

        newParams.linearAccelLimit.min = std::move(config.min_accel_x);
        newParams.linearAccelLimit.max = std::move(config.max_accel_x);

        newParams.angularAccelLimit.min = std::move(config.min_accel_z);
        newParams.angularAccelLimit.max = std::move(config.max_accel_z);

        newParams.linearJerkLimit.min = std::move(config.min_jerk_x);
        newParams.linearJerkLimit.max = std::move(config.max_jerk_x);
        
        newParams.angularJerkLimit.min = std::move(config.min_jerk_z);
        newParams.angularJerkLimit.max = std::move(config.max_jerk_z);

        m_DynamicParamsBuffer.writeFromNonRT(newParams);
    } 
}

PLUGINLIB_EXPORT_CLASS(
    hamal_diff_drive_controller::HamalDiffDriveController, controller_interface::ControllerBase
);