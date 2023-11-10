/**
 * @file emg_stop_node.cpp
 * @author your name (you@domain.com)
 * @brief Test node for EMG Stop.
 * @version 0.1
 * @date 2023-11-09
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "hamal_utils/emg_stop_client.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "emg_stop_test_node");
    ros::NodeHandle nh;
    EmgClientBase emgClient(nh);
    
    ros::Subscriber trigSub = nh.subscribe<std_msgs::Empty>("trigger_emg_stop_test", 1, [&](
        const std_msgs::Empty::ConstPtr& trig_msg){
            bool emgSent = emgClient.sendEmg();
            if(emgSent)
                ROS_INFO("EMG Stop signal sent.");
        }
    );

    ros::spin();

    return 0;

}