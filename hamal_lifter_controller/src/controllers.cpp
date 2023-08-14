/**
 * @file controllers.cpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-08-08
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "hamal_lifter_controller/controllers.hpp"

PID::PID()
{

}

PID::~PID()
{
    if(m_ActualValuePtr){
        delete m_ActualValuePtr;
    }
}

void PID::initController(
    const ros::Time& init_time,
    double Kp,
    double Ki,
    double Kd,
    double& actual_value
)
{
    m_PreviousTime = init_time;
    m_Kp = Kp;
    m_Ki = Ki;
    m_Kd = Kd;

    m_ActualValuePtr = &actual_value;
}


double PID::pid(
    const double& target_value,
    const ros::Time& current_time
)
{
    
    const double dt =  (current_time - m_PreviousTime).toSec();
    
    const double err =  target_value - *(m_ActualValuePtr); // Propotional

    m_SumOfErrors += err * dt; // Integral

    const double errorRate = (m_PreviousError - err) / dt; // Derivative 

    double output = 0.0;
    output = (m_Kp * err) + (m_Ki * m_SumOfErrors) + (m_Kd * errorRate);

    m_PreviousError = err;

    return output;

}