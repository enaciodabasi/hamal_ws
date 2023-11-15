#include <ros/ros.h>
#include <iostream>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "rate_test");

    ros::NodeHandle nh;
    ros::Rate r(500.0);

    ros::Time prev = ros::Time::now();
    while(ros::ok()){

        auto curr = ros::Time::now();
        auto elapsed = (curr - prev).toSec();
        ROS_INFO("Elapsed: %f [s]", elapsed);
        ros::spinOnce();    
        prev = curr;
        r.sleep();

    }

    ros::shutdown();
    return 0;

}