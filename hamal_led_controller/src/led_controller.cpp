#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/Twist.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <hamal_led_controller/Led.h>
#include <hamal_custom_interfaces/HardwareStatus.h>// Include the HardwareStatus message

ros::Publisher led_pub;
hamal_led_controller::Led m_led_status;

int taskMode;
float max_linear_vel = 1.0;
float max_angular_vel = 1.0;
bool emg_buffer = true;
bool lifter_buffer = false;

// void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
// {
//     if (msg->power_supply_status == sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING % m_led_status.mode != 2)
//     {
//         m_led_status.mode = 7;
//     }
// }

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{   
    if(m_led_status.mode == 1 || m_led_status.mode == 2){
        return;
    }
    if (msg->angular.z > max_angular_vel*0.75){
        m_led_status.mode = 5;
        m_led_status.speed = abs(msg->angular.z / max_angular_vel) * 100;
    }
    else if(msg->angular.z < -max_angular_vel * 0.75){
        m_led_status.mode = 6;
        m_led_status.speed = abs(-msg->angular.z / max_angular_vel) * 100;
    }
    else if(abs(max_linear_vel) < abs(msg->angular.z * 1.5)){
        m_led_status.mode = msg->angular.z > 0 ? 5 : 6;
        m_led_status.speed = abs(msg->angular.z / max_angular_vel) * 100;
    }
    else if(msg->linear.x != 0){
        m_led_status.mode = msg->linear.x > 0 ? 3 : 4;
        m_led_status.speed = abs(msg->linear.x / max_linear_vel) * 100;
    }
    else if(msg->angular.z != 0){
        m_led_status.mode = msg->angular.z > 0 ? 5 : 6;
        m_led_status.speed = abs(msg->angular.z / max_angular_vel) * 100;
    }
    else{
        m_led_status.mode = 0;
        m_led_status.speed = 0;
    }

    led_pub.publish(m_led_status);
}

void dockingCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg){
    if(m_led_status.mode != 2){
        if(msg->status_list[0].status==actionlib_msgs::GoalStatus::ACTIVE){
            m_led_status.mode = 1;
            led_pub.publish(m_led_status);
        }
        else{
            m_led_status.mode = 0;
            led_pub.publish(m_led_status);
        }
    }
}

void hardwareStatusCallback(const hamal_custom_interfaces::HardwareStatus::ConstPtr& msg)
{
    if (msg->slave_info.hardware_info_array[0].current_vel != 0 && msg->ec_system_status == 1){
        m_led_status.mode = 1;
        led_pub.publish(m_led_status);
        lifter_buffer = true;  
    }

    if (lifter_buffer && msg->ec_system_status == 1){
        if (msg->slave_info.hardware_info_array[0].current_vel == 0){
            m_led_status.mode = 0;
            led_pub.publish(m_led_status);
            lifter_buffer = false;  
        }
    }

    if (msg->ec_system_status == 0){
        m_led_status.mode = 2;
        led_pub.publish(m_led_status);
        emg_buffer = true; 
    }

    if (emg_buffer){
        if (msg->ec_system_status == 1){
            emg_buffer = false;
            m_led_status.mode = 0;
            led_pub.publish(m_led_status);
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "led_controller");
    ros::NodeHandle nh;

    led_pub = nh.advertise<hamal_led_controller::Led>("led_status", 10);
    ros::Subscriber cmd_vel_sub = nh.subscribe("/hamal/mobile_base_controller/cmd_vel", 10, cmdVelCallback);
    // ros::Subscriber battery_sub = nh.subscribe("/battery_state", 10, batteryCallback);
    ros::Subscriber autodock_status_sub = nh.subscribe("autodock_action/status", 10, dockingCallback);
    ros::Subscriber hardware_status_sub = nh.subscribe("/hamal/hardware_status", 10, hardwareStatusCallback); // Subscribe to the hardware status topic

    ros::Rate rate(60);  // Set the rate to 10 Hz

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
        led_pub.publish(m_led_status);
    }

    return 0;
}


