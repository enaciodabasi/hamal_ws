/**
 * @file velocity_multiplexer.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-02-07
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "hamal_velocity_multiplexer/velocity_multiplexer.hpp"

void parsePublisherInfoConfigFile(const YAML::Node& file, VelocityPublishers& publishers)
{
    VelocityPublishers info;
    
    auto subInfoNode = file["subscriber_info"];
    std::size_t idCount = 0;
    for(const auto& info_node : subInfoNode)
    {
        VelocityPublisherInfo info;
        
        info.id = idCount;
        idCount += 1; 
        info.name = info_node["name"].as<std::string>();
        info.prio = info_node["prio"].as<unsigned int>();
        info.allowedMode = RobotMode{info_node["allowed_mode"].as<uint16_t>()};
        info.topicName = info_node["topic_name"].as<std::string>();

        publishers[info.id] = info;
    }

}



int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "hamal_velocity_multiplexer");

    ros::NodeHandle nh;
    ros::Publisher outCmdVelPub = nh.advertise<geometry_msgs::Twist>(
        "/mobile_base_controller/cmd_vel",
        1,
        false
    );
    
    std::string subInfoConfigFilePath;
    nh.getParam("/hamal_velocity_multiplexer/sub_info_config_file", subInfoConfigFilePath);
    
    if(subInfoConfigFilePath.empty())
    {   
        ROS_ERROR("Can not find the config file for subscription info.");
        ros::shutdown();
    }

    double activityTimeoutPeriod = 0.0;
    nh.getParam("/hamal_velocity_multiplexer/publisher_activity_timeout_period", activityTimeoutPeriod);
    VelocityPublishers pubsInfo;
    parsePublisherInfoConfigFile(YAML::LoadFile(subInfoConfigFilePath), pubsInfo);
    const uint32_t threadCountPerPub = pubsInfo.size();
    ros::MultiThreadedSpinner mtSpinner(threadCountPerPub + 1);

    /* auto cmdVelCallback = [&](const geometry_msgs::Twist& msg, unsigned int id)
    {

    }; */

    bool cmdVelMultiplexerActive = false;

    ros::ServiceClient robotModeClient = nh.serviceClient<hamal_custom_interfaces::GetRobotMode>(
        "/get_robot_mode",
        true
    );
    
    ros::Rate rate(50);
    while(!robotModeClient.waitForExistence())
    {
        ROS_WARN("Waiting for robot mode server to come up.");
        rate.sleep();
    }

    RobotMode currentRobotMode{0};

    unsigned int activePubID = 0x0;

    boost::function<void (const geometry_msgs::Twist::ConstPtr& , unsigned int&)> cmdVelCallback = 
    [&](const geometry_msgs::Twist::ConstPtr& msg, unsigned int id)
    {
        hamal_custom_interfaces::GetRobotModeRequest req;
        hamal_custom_interfaces::GetRobotModeResponse rep;

        bool robotModeCallSucc = robotModeClient.call(req, rep);

        const auto currentMode = rep.mode;

        currentRobotMode = RobotMode{currentMode};

        /**
         * Simplify the logic, add timeout check.
         * 
         */
        if(cmdVelMultiplexerActive)
        {
            if(currentRobotMode == RobotMode::Manual && pubsInfo.at(id).allowedMode == currentRobotMode)
            {
                if(activePubID == id)
                {
                    outCmdVelPub.publish(msg);
                }
                else if(activePubID != id) // Try switch
                {
                    if(pubsInfo.at(id).prio > pubsInfo.at(activePubID).prio) // Switch
                    {
                        outCmdVelPub.publish(msg);
                        pubsInfo.at(activePubID).isActive = false;
                        pubsInfo.at(id).isActive = true;
                        activePubID = id;    
                    }
                    else
                    {
                        return;
                    }
                }

            }
            else if(currentRobotMode == RobotMode::Autonomous && pubsInfo.at(id).allowedMode == currentRobotMode)
            {
                if(activePubID == id)
                {
                    outCmdVelPub.publish(msg);
                }
                else if(activePubID != id) // Try switch
                {
                    if(pubsInfo.at(id).prio > pubsInfo.at(activePubID).prio) // Switch 
                    {
                        outCmdVelPub.publish(msg);
                        pubsInfo.at(activePubID).isActive = false;
                        pubsInfo.at(id).isActive = true;
                        activePubID = id;    
                    }
                    else
                    {
                        return;
                    }
                }
            }
            else if(currentRobotMode == RobotMode::Hybrid && pubsInfo.at(id).allowedMode == currentRobotMode)
            {
                if(activePubID == id)
                {
                    outCmdVelPub.publish(msg);
                }
                else if(activePubID != id) // Try switch
                {
                    if(pubsInfo.at(id).prio > pubsInfo.at(activePubID).prio) // Switch
                    {
                        outCmdVelPub.publish(msg);
                        pubsInfo.at(activePubID).isActive = false;
                        pubsInfo.at(id).isActive = true;
                        activePubID = id;    
                    }
                    else
                    {
                        return;
                    }
                }
            }
        }
        else
        {
            cmdVelMultiplexerActive = true;
            pubsInfo.at(id).isActive = true;
            activePubID = id;
            outCmdVelPub.publish(msg);

        }
        

    }; 


    for(auto& [key, value] : pubsInfo)
    {
        value.cmdVelSub = nh.subscribe<geometry_msgs::Twist>(
            value.topicName,
            10,
            boost::bind(cmdVelCallback, _1, value.id)
        );
    }

    mtSpinner.spin();

    ros::shutdown();
    
    return 0;
    
}

