/**
 * @file hamal_diff_drive_controller.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef HAMAL_DIFF_DRIVE_CONTROLLER_
#define HAMAL_DIFF_DRIVE_CONTROLLER_

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <pluginlib/class_list_macros.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/tfMessage.h>
#include <tf/tf.h>

#include <boost/thread/recursive_mutex.hpp>
#include <memory>
#include <mutex>

#include "hamal_diff_drive_controller/defs.hpp"
#include "hamal_diff_drive_controller/odometry.hpp"
#include "hamal_diff_drive_controller/limiter.hpp"
#include "hamal_diff_drive_controller/HamalDiffDriveControllerConfig.h"

namespace hamal_diff_drive_controller
{

    struct WheelHandle
    {
        hardware_interface::JointHandle jointHandle;
        double previousVelocity;
        double desiredVelocity;
    };

    struct VelCommand
    {
        VelCommand() : timestamp(0.0) {linear = 0.0; angular=0.0;}
        double linear;
        double angular;
        ros::Time timestamp;
    };

    class HamalDiffDriveController : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
    {
        public:

        HamalDiffDriveController();

        bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& base_nh, ros::NodeHandle& controller_nh);

        void update(const ros::Time& time, const ros::Duration& period);

        void starting(const ros::Time& time);

        void stopping(const ros::Time& time);

        private:

        std::string m_ControllerName;

        WheelHandle m_LeftWheel;
        
        WheelHandle m_RightWheel;

        std::unique_ptr<PositionOdometry> m_PosOdom;

        std::unique_ptr<VelocityOdometry> m_VelOdom;

        ros::Time m_PreviousTime;
        ros::Time m_OdomControlTime;

        std::string m_TwistCmdTopicName = "/cmd_vel"; 
        ros::Subscriber m_TwistCmdSub;
        realtime_tools::RealtimeBuffer<VelCommand> m_VelCommandBuffer;
        VelCommand m_VelCommand;
        
        double m_PublishRate = 50.0;
        ros::Duration m_PublishPeriod;

        bool m_OdomTfEnabled = true;

        bool m_UsePositionForOdom = false;

        bool m_UseVelocityForOdom = false;

        bool m_UseBothOdom = true;

        /* bool m_PublishUnfilteredOdom = false; */
        
        std::string m_BaseFrameID = "base_link";
         
        std::string m_OdomFrameID = "odom"; 
        /* std::string m_FilteredOdomTopicName = "/filtered_odom"; */
        std::string m_UnfilteredPosOdomTopicName = "/unfiltered_position_odom";
        std::string m_UnfilteredVelOdomTopicName = "/unfiltered_velocity_odom";

        /* std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> m_FilteredOdomPublisher; */
        
        std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> m_UnfilteredPositionOdomPub;
        
        std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> m_UnfilteredVelocityOdomPub;
        
        std::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage>> m_OdomTfPublisher;

        std::shared_ptr<realtime_tools::RealtimePublisher<control_msgs::JointTrajectoryControllerState> > m_ControllerStatePublister;

        WheelParams m_WheelParams;

        double m_VelCommandTimeout = 0.5;
        
        bool m_EnableMultipleCmdVelPublishers;

        std::unique_ptr<Limiter> m_LinearVelLimiter;
        
        std::unique_ptr<Limiter> m_AngularVelLimiter;

        std::pair<VelCommand, VelCommand> m_LastTwoVelCmds;

        struct DynamicParams
        {
            Limit linearVelLimit;
            Limit angularVelLimit;

            Limit linearAccelLimit;
            Limit angularAccelLimit;

            Limit linearJerkLimit;
            Limit angularJerkLimit;

            double publish_rate;

            DynamicParams()
            {
                linearVelLimit = Limit();
                angularVelLimit = Limit();
                publish_rate = 50.0;
            }

        };

        realtime_tools::RealtimeBuffer<DynamicParams> m_DynamicParamsBuffer;

        boost::recursive_mutex m_DynamicParamMutex;

        std::shared_ptr<dynamic_reconfigure::Server<HamalDiffDriveControllerConfig>> m_DynamicParamServer;

        void twistCmdCallback(const geometry_msgs::Twist& twist_cmd);

        void dynamicReconfigureCallback(const HamalDiffDriveControllerConfig& config, uint32_t);

    };

}


#endif // HAMAL_DIFF_DRIVE_CONTROLLER_