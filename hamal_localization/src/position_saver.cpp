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
    const geometry_msgs::PoseStamped& pose_msg
)
{
    existing_yaml_node["robot_position"]["frame_id"] = pose_msg.header.frame_id;
    existing_yaml_node["robot_position"]["position"]["x"] = pose_msg.pose.position.x;
    existing_yaml_node["robot_position"]["position"]["y"] = pose_msg.pose.position.y;
    existing_yaml_node["robot_position"]["position"]["z"] = pose_msg.pose.position.z;
    existing_yaml_node["robot_position"]["orientation"]["x"] = pose_msg.pose.orientation.x;
    existing_yaml_node["robot_position"]["orientation"]["y"] = pose_msg.pose.orientation.y;
    existing_yaml_node["robot_position"]["orientation"]["z"] = pose_msg.pose.orientation.z;
    existing_yaml_node["robot_position"]["orientation"]["w"] = pose_msg.pose.orientation.w;

    /* YAML::Node covarianceNode;
    for(std::size_t covarianceMatIter = 0; covarianceMatIter < pose_msg.pose.covariance.size(); covarianceMatIter++)
    {
        covarianceNode.push_back(pose_msg.pose.covariance.at(covarianceMatIter));
    }

    existing_yaml_node["robot_position"]["covariance"] = covarianceNode; */

}

const geometry_msgs::PoseStamped turnYamlNodeToPose(
  const YAML::Node& existing_yaml_node
)
{
    
    geometry_msgs::PoseStamped poseMsg;
    
    poseMsg.header.frame_id = existing_yaml_node["robot_position"]["frame_id"].as<std::string>();
    poseMsg.pose.position.x = existing_yaml_node["robot_position"]["position"]["x"].as<double>();
    poseMsg.pose.position.y = existing_yaml_node["robot_position"]["position"]["y"].as<double>();
    poseMsg.pose.position.z = existing_yaml_node["robot_position"]["position"]["z"].as<double>();
    poseMsg.pose.orientation.x = existing_yaml_node["robot_position"]["orientation"]["x"].as<double>();
    poseMsg.pose.orientation.y = existing_yaml_node["robot_position"]["orientation"]["y"].as<double>();
    poseMsg.pose.orientation.z = existing_yaml_node["robot_position"]["orientation"]["z"].as<double>(); 
    poseMsg.pose.orientation.w = existing_yaml_node["robot_position"]["orientation"]["w"].as<double>();

    /* auto covarianceMatrix = existing_yaml_node["robot_position"]["covariance"].as<std::vector<double>>();

    auto covArrOpt = [&covarianceMatrix]() -> std::optional<boost::array<double, 36>> {
        if(covarianceMatrix.size() == 36){
            boost::array<double, 36> covArr;
            boost::range::copy(covarianceMatrix, covArr.begin());
            return covArr;
        }

        return std::nullopt; 
    }();
    
    if(covArrOpt){
        poseMsg.pose.covariance = covArrOpt.value();
    } */

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
    /* ros::Subscriber poseSub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
        "amcl_pose",
        1,
        [&](const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose){
            poseData = *pose;
            
        }
    ); */

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener(tfBuffer, nh, true);

    ros::Publisher initialPosePub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        "/initialpose",
        1,
        true
    );
    std::string user = getlogin();
    const std::string pathToConfig = "/home/" + user + "/hamal_ws/src/hamal_localization/pos_logs/pos_log.yaml";
    bool isInitialPosSaving = false;
    if(!std::filesystem::exists(pathToConfig)){
        std::ofstream newFile(pathToConfig);

        YAML::Node emptyFileNode;
        emptyFileNode["robot_position"]["frame_id"] = "map";
        emptyFileNode["robot_position"]["position"]["x"] = 0.0;
        emptyFileNode["robot_position"]["position"]["y"] = 0.0;
        emptyFileNode["robot_position"]["position"]["z"] = 0.0;
        emptyFileNode["robot_position"]["orientation"]["x"] = 0.0;
        emptyFileNode["robot_position"]["orientation"]["y"] = 0.0;
        emptyFileNode["robot_position"]["orientation"]["z"] = 0.0;
        emptyFileNode["robot_position"]["orientation"]["w"] = 0.0;

        /* YAML::Node covarianceNode;
        for(std::size_t covarianceMatIter = 0; covarianceMatIter < 36; covarianceMatIter++)
        {
            covarianceNode.push_back(0.0);
        }

        emptyFileNode["robot_position"]["covariance"] = covarianceNode; */
        isInitialPosSaving = true;
    }



    YAML::Node baseFile = YAML::LoadFile("/home/" + user + "/hamal_ws/src/hamal_localization/pos_logs/pos_log.yaml");

    ros::Rate rate(100.0);
    ros::Rate checkSubRate(500.0);
    bool initialPosePublished = false;
    if(!isInitialPosSaving){
        while(initialPosePub.getNumSubscribers() == 0)
        {   
            if(ros::ok())
                ros::spinOnce();
            checkSubRate.sleep();
        }

        if(initialPosePub.getNumSubscribers() != 0){
            ROS_INFO("Publishing to /initialpose");
            geometry_msgs::PoseWithCovarianceStamped poseToPub;
            auto cpyPose = turnYamlNodeToPose(baseFile);
            poseToPub.header = cpyPose.header;
            poseToPub.pose.pose = cpyPose.pose;
            initialPosePub.publish(poseToPub);
            initialPosePublished = true;
        }
    }
    else{
        initialPosePublished = true;
    }

    auto tfToPoseStamped = [](geometry_msgs::PoseStamped& pose, geometry_msgs::TransformStamped tf)->void{
        
        pose.pose.position.x = tf.transform.translation.x;
        pose.pose.position.y = tf.transform.translation.y;
        pose.pose.position.z = tf.transform.translation.z;
        pose.header.frame_id = tf.header.frame_id;

        pose.pose.orientation = tf.transform.rotation;
    };
    while(ros::ok())
    {
        if(!initialPosePublished){
            rate.sleep();
            continue;
        }
        ros::spinOnce();
        //if(!positionArrived){
        //    continue;
        //}
        geometry_msgs::PoseStamped robotPoseRelativeToMap;
        try{
            auto robotToMap = tfBuffer.lookupTransform("map", "base_link", ros::Time(0.0));
            
            tfToPoseStamped(robotPoseRelativeToMap, robotToMap);
        }catch(...){
            rate.sleep();
            continue;
        }
        
        turnPoseMsgToYamlNode(baseFile, robotPoseRelativeToMap);
        
        std::ofstream out;
        out.open("/home/"+ user +"/hamal_ws/src/hamal_localization/pos_logs/pos_log.yaml");
        out << baseFile; 

        if(out.is_open()){
            out.close();
        }
        
        rate.sleep();
    }

    
    ros::waitForShutdown();

    return 0;

}