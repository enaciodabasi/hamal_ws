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

void turnPoseMsgToYamlNode(
    YAML::Node& existing_yaml_node,
    const geometry_msgs::PoseWithCovarianceStamped& pose_msg
)
{
    existing_yaml_node["robot_position"]["frame_id"] = pose_msg.header.frame_id;
    existing_yaml_node["robot_position"]["position"]["x"] = pose_msg.pose.pose.position.x;
    existing_yaml_node["robot_position"]["position"]["y"] = pose_msg.pose.pose.position.y;
    existing_yaml_node["robot_position"]["position"]["z"] = pose_msg.pose.pose.position.z;
    existing_yaml_node["robot_position"]["orientation"]["x"] = pose_msg.pose.pose.orientation.x;
    existing_yaml_node["robot_position"]["orientation"]["y"] = pose_msg.pose.pose.orientation.y;
    existing_yaml_node["robot_position"]["orientation"]["z"] = pose_msg.pose.pose.orientation.z;
    existing_yaml_node["robot_position"]["orientation"]["w"] = pose_msg.pose.pose.orientation.w;

    YAML::Node covarianceNode;
    for(std::size_t covarianceMatIter = 0; covarianceMatIter < pose_msg.pose.covariance.size(); covarianceMatIter++)
    {
        covarianceNode.push_back(pose_msg.pose.covariance.at(covarianceMatIter));
    }

    existing_yaml_node["robot_position"]["covariance"] = covarianceNode;

}

const geometry_msgs::PoseWithCovarianceStamped turnYamlNodeToPose(
  const YAML::Node& existing_yaml_node
)
{
    geometry_msgs::PoseWithCovarianceStamped poseMsg;
    poseMsg.header.frame_id = existing_yaml_node["robot_position"]["frame_id"].as<std::string>();
    poseMsg.pose.pose.position.x = existing_yaml_node["robot_position"]["position"]["x"].as<double>();
    poseMsg.pose.pose.position.y = existing_yaml_node["robot_position"]["position"]["y"].as<double>();
    poseMsg.pose.pose.position.z = existing_yaml_node["robot_position"]["position"]["z"].as<double>();
    poseMsg.pose.pose.orientation.x = existing_yaml_node["robot_position"]["orientation"]["x"].as<double>();
    poseMsg.pose.pose.orientation.y = existing_yaml_node["robot_position"]["orientation"]["y"].as<double>();
    poseMsg.pose.pose.orientation.z = existing_yaml_node["robot_position"]["orientation"]["z"].as<double>(); 
    poseMsg.pose.pose.orientation.w = existing_yaml_node["robot_position"]["orientation"]["w"].as<double>();

    return poseMsg;
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "robot_position_saver");
    std::string pathToPosFile;

    ros::NodeHandle nh;
    if(nh.hasParam("pos_saver/position_file_path")){
        nh.getParam("pos_saver/position_file_path", pathToPosFile);
    }
    
    bool positionArrived = false;
    geometry_msgs::PoseWithCovarianceStamped poseData;
    ros::Subscriber poseSub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
        "amcl_pose",
        1,
        [&](const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose){
            poseData = *pose;
            
        }
    );

    ros::Publisher initialPosePub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        "initialpose",
        1,
        true
    );
    YAML::Node baseFile = YAML::LoadFile("/home/grass/amr_ws/src/hamal_localization/pos_logs/pos_log.yaml");

    ros::Rate rate(10.0);
    ros::Rate checkSubRate(50.0);
    while(initialPosePub.getNumSubscribers() == 0)
    {   
        ROS_INFO("No subs for initial pose.");
    }

    if(initialPosePub.getNumSubscribers() != 0){
        initialPosePub.publish(turnYamlNodeToPose(baseFile));
    }

    while(ros::ok())
    {
        ros::spinOnce();
        //if(!positionArrived){
        //    continue;
        //}
        turnPoseMsgToYamlNode(baseFile, poseData);
        
        std::ofstream out;
        out.open("/home/grass/amr_ws/src/hamal_localization/pos_logs/pos_log.yaml");
        out << baseFile;

        if(out.is_open()){
            out.close();
        }
        
        rate.sleep();
    }

    
    ros::waitForShutdown();

    return 0;

}