/**
 * @file lifter_trajectory_generator.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-07-06
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "hamal_control/lifter_trajectory_generator.hpp"

/* const Profiles generateProfiles()
{

}

const Profile positionProfile(const Constraints& constraints)
{

}

const Profile velocityProfile()
{

}

const Profile accelerationProfile()
{

} */

LifterControllerClient::LifterControllerClient(ros::NodeHandle& nh, const std::string& action_server_name, const std::string& lifter_joint_name)
    : m_Node(nh), m_FollowJointTrajClient(nh, action_server_name, true), m_LifterJointName{lifter_joint_name}
{

}

bool LifterControllerClient::init()
{
    bool serverOk = m_FollowJointTrajClient.waitForServer(ros::Duration(5.0));
    if(!serverOk)
    {
        ROS_ERROR("Can't find action server");
    }
    
    m_NewLifterCmdSub = m_Node.subscribe(
        "/hamal/lifter/move_goal",
        10,
        &LifterControllerClient::newCommandCallback,
        this
    );

    m_CancelGoalSub = m_Node.subscribe(
        "/hamal/lifter/cancel_goal", 
        1, 
        &LifterControllerClient::cancelGoalCallback, 
        this);

    return true;
}

bool LifterControllerClient::cancelActionGoal()
{
    m_FollowJointTrajClient.cancelGoal();
}

void LifterControllerClient::newCommandCallback(const hamal_custom_interfaces::LifterCommand &lifter_cmd)
{

    if(!m_FollowJointTrajClient.isServerConnected())
    {
        return;
    }

    auto newCmd = std::move(lifter_cmd);

    if(newCmd.override_latest_command)  
    {
        m_FollowJointTrajClient.cancelGoal();
    }
    
    control_msgs::FollowJointTrajectoryGoal goal;
    
    goal.trajectory.header.frame_id = "base_link";
    /* goal.trajectory.header.stamp = ros::Time::now(); */
    //goal.trajectory.header.frame_id = m_LifterJointName;
    goal.trajectory.joint_names.resize(1);
    goal.trajectory.joint_names.at(0) = m_LifterJointName;

    goal.trajectory.points.resize(2);    
    auto& initial_point = goal.trajectory.points.at(0);
    initial_point.positions.resize(1);
    initial_point.positions.at(0) = newCmd.initial_position;
    initial_point.velocities.resize(1);
    initial_point.velocities.at(0) = newCmd.initial_velocity;
    initial_point.accelerations.resize(1);
    initial_point.accelerations.at(0) = newCmd.initial_acceleration;

    auto& final_point = goal.trajectory.points.at(1);
    final_point.positions.resize(1);
    final_point.positions.at(0) = newCmd.final_position;
    final_point.velocities.resize(1);
    final_point.velocities.at(0) = newCmd.final_velocity;
    final_point.accelerations.resize(1);
    final_point.accelerations.at(0) = newCmd.final_acceleration;
    goal.goal_time_tolerance = newCmd.desired_execution_time;
    
    m_FollowJointTrajClient.sendGoal(
        goal,
        boost::bind(&LifterControllerClient::goalDoneCallback, this, boost::placeholders::_1, boost::placeholders::_2),
        ControllerClient::SimpleActiveCallback(),
        ControllerClient::SimpleFeedbackCallback()
    );


}

void LifterControllerClient::feedbackCallback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
{

}

void LifterControllerClient::goalDoneCallback(const actionlib::SimpleClientGoalState &state, const control_msgs::FollowJointTrajectoryResultConstPtr &result)
{

}

void LifterControllerClient::cancelGoalCallback(const std_msgs::Empty &cancel)
{
    cancelActionGoal();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lifter_controller_client");
    ros::NodeHandle nh;
    LifterControllerClient lifterControllerClient(nh, "/hamal/lifter_controller/follow_joint_trajectory", "lifter_joint");
    bool ok = lifterControllerClient.init();
    if(!ok)
        return -1;

    ros::spin();

    ros::shutdown();

    return 0;
}