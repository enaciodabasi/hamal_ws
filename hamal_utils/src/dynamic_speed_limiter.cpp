/**
 * @file dynamic_speed_limiter.cpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-10-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "hamal_utils/dynamic_speed_limiter.hpp"

template<class Config>
//using SyncFunc = std::function<void(ros::NodeHandle&, Config& config)>;
using SyncFunc = std::function<void()>;
static bool changeSpeedParams = false;

teb_local_planner::TebLocalPlannerReconfigureConfig* tebConfPtr;

diff_drive_controller_hamal::DiffDriveControllerHamalConfig* controllerConfPtr;

SyncFunc<teb_local_planner::TebLocalPlannerReconfigureConfig> syncTebParamsCb;
SyncFunc<diff_drive_controller_hamal::DiffDriveControllerHamalConfig> syncControllerParamsCb;

static VelParams initialVelParams;

void speedLimitRatioCallback(const SpeedLimitRatio::ConstPtr& speed_limit_ratio)
{   

    if(!tebConfPtr || !controllerConfPtr){
        return;
    }

    //syncTebParamsCb();
    //syncControllerParamsCb();

    auto& tebConf = *tebConfPtr;
    auto& controllerConf = *controllerConfPtr;

    const double multiplier = (speed_limit_ratio->ratio) / 100.0;

    tebConf.max_vel_x = initialVelParams.max_vel_x * multiplier;
    tebConf.max_vel_x_backwards = (-1.0 ) * initialVelParams.min_vel_x * multiplier;
    tebConf.max_vel_y =  0.0;
    tebConf.max_vel_theta =  initialVelParams.max_vel_z * multiplier;

    tebConf.acc_lim_x =  initialVelParams.max_acc_x * multiplier;
    tebConf.acc_lim_y = 0.0;
    tebConf.acc_lim_theta = initialVelParams.max_acc_z * multiplier;

    controllerConf.max_vel_x = initialVelParams.max_vel_x * multiplier;
    controllerConf.min_vel_x = initialVelParams.min_vel_x * multiplier;

    controllerConf.max_vel_z = initialVelParams.max_vel_z * multiplier;
    controllerConf.min_vel_z = initialVelParams.min_vel_z * multiplier;
    
    controllerConf.max_acc_x = initialVelParams.max_acc_x * multiplier;
    controllerConf.min_acc_x = initialVelParams.min_acc_x * multiplier;

    controllerConf.max_acc_z = initialVelParams.max_acc_z * multiplier;
    controllerConf.min_acc_z = initialVelParams.min_acc_z * multiplier;

    controllerConf.max_jerk_x = initialVelParams.max_jerk_x * multiplier;
    controllerConf.min_jerk_x = initialVelParams.min_jerk_x * multiplier;

    controllerConf.max_jerk_z = initialVelParams.max_jerk_z * multiplier;
    controllerConf.min_jerk_z = initialVelParams.min_jerk_z * multiplier;

    dynamic_reconfigure::Client<diff_drive_controller_hamal::DiffDriveControllerHamalConfig> controllerConfigClient(
        "/hamal/mobile_base_controller",
        [&](const diff_drive_controller_hamal::DiffDriveControllerHamalConfig&){
            ROS_INFO("diff_drive_controller_hamal velocity parameters have been changed");
        },
        [&](const dynamic_reconfigure::ConfigDescription&){
            return;
        }
    );  

    dynamic_reconfigure::Client<teb_local_planner::TebLocalPlannerReconfigureConfig> tebConfigClient(
        "/move_base/TebLocalPlannerROS",
        [&](const teb_local_planner::TebLocalPlannerReconfigureConfig&){
            ROS_INFO("diff_drive_controller_hamal velocity parameters have been changed");
        },
        [&](const dynamic_reconfigure::ConfigDescription&){
            return;
        }
    );

    tebConfigClient.setConfiguration(tebConf);
    controllerConfigClient.setConfiguration(controllerConf);
    

}

/* void syncTebParams(ros::NodeHandle& nh, teb_local_planner::TebLocalPlannerReconfigureConfig& teb_config)
{
    
    //const std::string tebNs{"/teb_local_planner/"};
    teb_local_planner::TebLocalPlannerReconfigureConfig tempConfig;
    tempConfig.__fromServer__(nh);
    teb_config = std::move(tempConfig);

}

void syncControllerParams(ros::NodeHandle& nh, diff_drive_controller_hamal::DiffDriveControllerHamalConfig& controller_config)
{
    diff_drive_controller_hamal::DiffDriveControllerHamalConfig tempConfig;
    tempConfig.__fromServer__(nh);
    controller_config = std::move(tempConfig);
} */

int main(int argc, char** argv)
{

    ros::init(argc, argv, "dynamic_speed_limiter");
    
    teb_local_planner::TebLocalPlannerReconfigureConfig tebConf;
    diff_drive_controller_hamal::DiffDriveControllerHamalConfig controllerConf;

    tebConfPtr = &tebConf;
    controllerConfPtr = &controllerConf;

    bool tebInitialConf, controllerInitialConfig = true;

    ros::NodeHandle nh;
    ros::Subscriber ratioSub = nh.subscribe(
        "/speed_limit_ratio",
        1,
        &speedLimitRatioCallback
    );

    ros::Subscriber tebConfigUpdateSub = nh.subscribe<dynamic_reconfigure::Config>(
        "/move_base/TebLocalPlannerROS/parameter_updates",
        1,
        [&](const dynamic_reconfigure::Config::ConstPtr& config){
            auto conf = *config;
            tebConf.__fromMessage__(conf);
            if(tebInitialConf){
                
                initialVelParams.max_vel_x = tebConf.max_vel_x;
                initialVelParams.min_vel_x = -1.0 * tebConf.max_vel_x_backwards;
                
                initialVelParams.max_vel_z = tebConf.max_vel_theta;
                initialVelParams.min_vel_z = -initialVelParams.max_vel_z;

                initialVelParams.max_acc_x = tebConf.acc_lim_x;
                
                initialVelParams.max_acc_z = tebConf.acc_lim_theta;

                tebInitialConf = false;
            }
        }
    );
    
    ros::Subscriber controllerConfigSub = nh.subscribe<dynamic_reconfigure::Config>(
        "/hamal/mobile_base_controller/parameter_updates",
        1,
        [&](const dynamic_reconfigure::Config::ConstPtr& config){
            auto conf = *config;
            controllerConf.__fromMessage__(conf);
            if(controllerInitialConfig){

                initialVelParams.min_acc_x = controllerConf.min_acc_x;
                initialVelParams.min_acc_z = controllerConf.min_acc_z;
                
                initialVelParams.min_vel_x = controllerConf.min_vel_x;
                initialVelParams.min_vel_z = controllerConf.min_vel_z;

                initialVelParams.max_jerk_x = controllerConf.max_jerk_x;
                initialVelParams.max_jerk_z = controllerConf.max_jerk_z;

                initialVelParams.min_jerk_x = controllerConf.min_jerk_x;
                initialVelParams.min_jerk_z = controllerConf.min_jerk_z;

                controllerInitialConfig = false;
            }
        }
    );

    

/*     syncTebParamsCb = std::bind(syncTebParams, nh, tebConf);
    syncControllerParamsCb = std::bind(syncControllerParams, nh, controllerConf); */

    while(ros::ok()){

        ros::spinOnce();
    }
    //if(tebConfPtr)
    //    delete tebConfPtr;
    //if(controllerConfPtr)
    //    delete controllerConfPtr;

    tebConfPtr = nullptr;
    controllerConfPtr = nullptr;

    return 0;
}