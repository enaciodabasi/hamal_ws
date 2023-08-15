/**
 * @file controllers.hpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-08-08
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#include <ros/time.h>


#ifndef CONTROLLERS_HPP_
#define CONTROLLERS_HPP_

class PID
{
    public:
    
    PID();

    ~PID();

    void initController(
        const ros::Time& init_time,
        double Kp,
        double Ki,
        double Kd    
    );
    
    double pid(
        const double& target_value,
        const double& current_value,
        const ros::Time& current_time
    );

    long double errorFunction(
        const double& target_value,
        const double& actual_value
    );
    private:
    
/*     double* m_ActualValuePtr;
 */    
    double m_Error;
    double m_PreviousError;
    double m_SumOfErrors;

    double m_Kp;

    double m_Ki;

    double m_Kd;

    ros::Time m_PreviousTime;

};

#endif // CONTROLLERS_HPP_