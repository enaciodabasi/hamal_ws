/**
 * @file position_saver.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-10-04
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "hamal_localization/position_saver.hpp"

void turnPoseMsgToYAML(
    YAML::Node& existing_yaml_node,
    const geometry_msgs::PoseWithCovarianceStamped& pose_msg
)
{
    existing_yaml_node["robot_position"]["frame_id"] = pose_msg.header.frame_id;
    existing_yaml_node["robot_position"]["position"]["x"] = pose_msg.pose.pose.position.x;
    existing_yaml_node["robot_position"]["position"]["y"] = pose_msg.pose.pose.position.y;
    existing_yaml_node["robot_position"]["position"]["z"] = pose_msg.pose.pose.position.z;
    existing_yaml_node["robot_position"]["orientation"]["x"] = pose_msg.pose.pose.orientation.x;
    existing_yaml_node["robot_orientation"]["orientation"]["y"] = pose_msg.pose.pose.orientation.y;
    existing_yaml_node["robot_orientation"]["orientation"]["z"] = pose_msg.pose.pose.orientation.z;
    existing_yaml_node["robot_orientation"]["orientation"]["w"] = pose_msg.pose.pose.orientation.w;

}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "robot_position_saver");
    std::string pathToPosFile;

    ros::NodeHandle nh;
    if(nh.hasParam("position_file_path")){
        nh.getParam("position_file_path", pathToPosFile);
    }
    geometry_msgs::PoseWithCovarianceStamped poseData;
    ros::Subscriber poseSub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
        "amcl_pose",
        1,
        [&](const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose){
            poseData = *pose;
        }
    );
    YAML::Node baseFile = YAML::LoadFile(pathToPosFile);

    ros::Rate rate(10.0);

    while(ros::ok())
    {
        ros::spinOnce();
        turnPoseMsgToYAML(baseFile, poseData);
        
        std::ofstream out;
        out.open(pathToPosFile);
        out << baseFile;

        if(out.is_open()){
            out.close();
        }
        
        rate.sleep();
    }

    
    ros::waitForShutdown();

    return 0;

}