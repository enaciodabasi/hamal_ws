/**
 * @file lifter_trajectory_generator.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-07-05
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef LIFTER_TRAJECTORY_GENERATOR_HPP_
#define LIFTER_TRAJECTORY_GENERATOR_HPP_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/Empty.h>

#include "hamal_custom_interfaces/LifterCommand.h"

#include <vector>
#include <iostream>
#include <boost/bind.hpp>

using ControllerClient = actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;

class LifterControllerClient
{
    public:

    LifterControllerClient(ros::NodeHandle& nh, const std::string& action_server_name, const std::string& lifter_joint_name);

    bool init();

    private:

    ros::NodeHandle m_Node;

    ControllerClient m_FollowJointTrajClient;

    ros::Subscriber m_NewLifterCmdSub;

    ros::Subscriber m_CancelGoalSub;

    std::string m_LifterJointName;

    // Private Functions

    bool cancelActionGoal();

    // ROS callbacks

    void newCommandCallback(const hamal_custom_interfaces::LifterCommand& lifter_cmd);

    void goalIsActiveCallback()
    {

    }

    void feedbackCallback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback);

    void goalDoneCallback(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr& result);

    void cancelGoalCallback(const std_msgs::Empty& cancel);

};

/* using Profile = std::vector<double>;
struct Constraints
{
    double t_max;
    double v_max;
    double a_max;

    Constraints() : t_max(0.0), v_max(0.0), a_max(0.0) {}

};

struct Profiles
{
    Profile positionProfile;
    Profile velocityProfile;
    Profile accelerationProfile;
};
 */
/*
    q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t5
    - T: Difference between the maximum time and current time
    - qf: Desired final value of q
    - qi: Initial value of q
    - a0: initial value of q = q
    - a1: initial value of the derivative of q = q'
    - a2: initial value of the 2nd order derivative of q divided by 2 = q''
    - a3: 1/2*T^3[20(qf − qi) − (8q'f + 12 q'i)T − (3q''f − q''i)T^2]
    - a4: 1/2*T^4[30(qf − qi) + (14*q'f + 16*q'i)*T + (3*q''f − 2*q'2i)*T^2]
    - a5: 1/2*T^5[12(qf - qi) - 6(q'f + q'i)*T + (q''f - q''i)*T^2]
*/

/* const Profiles generateProfiles(
    const Constraints& constraints,
    double initial_position,
    double desired_position
);

const Profile positionProfile();

const Profile velocityProfile();

const Profile accelerationProfile(); */
 

#endif // LIFTER_TRAJECTORY_GENERATOR_HPP_