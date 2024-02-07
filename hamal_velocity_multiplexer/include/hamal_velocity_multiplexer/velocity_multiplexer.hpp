/**
 * @file velocity_multiplexer.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-02-07
 * 
 * @copyright Copyright (c) 2024
 * 
 */


#include <map>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <hamal_custom_interfaces/GetRobotMode.h>

#include <yaml-cpp/yaml.h>

enum class RobotMode : unsigned int
{
    Manual = 0,
    Autonomous = 1,
    Hybrid = 2
};

struct VelocityPublisherInfo
{
    unsigned int id;
    
    unsigned int prio;

    std::string name;

    std::string topicName;

    bool isActive;

    RobotMode allowedMode;

    bool init();

    ros::Subscriber cmdVelSub;

};



using VelocityPublishers = std::map<unsigned int, VelocityPublisherInfo>;

void parsePublisherInfoConfigFile(const YAML::Node& file, VelocityPublishers& publishers);

