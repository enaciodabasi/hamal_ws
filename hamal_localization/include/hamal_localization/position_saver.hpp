/**
 * @file position_saver.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-10-04
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef POSITION_SAVER_HPP_
#define POSITION_SAVER_HPP_

#include <unordered_map>
#include <filesystem>
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <yaml-cpp/yaml.h>

/**
 * @brief Holds data name and value pairs
 * frame_id: Name of the frame
 * position: position of the robot
 *  x:
 *  y:
 *  z:
 * orientation: orientation of the robot
 * x:
   y:
   z:
   w: 
 */

void turnPoseMsgToYAML(
    YAML::Node& existing_yaml_node,
    const geometry_msgs::PoseWithCovarianceStamped& pose_msg
);

#endif