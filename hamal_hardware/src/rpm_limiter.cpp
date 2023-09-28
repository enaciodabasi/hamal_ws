#include "hamal_hardware/rpm_limiter.hpp"

rpm_limiter::rpm_limiter() {
         maxVel = (0.0); 
         minVel = (0.0); 
         maxAcc = (0.0); 
         minAcc = (0.0); 
         previousVel = (0.0);
}
rpm_limiter::rpm_limiter(
        const double& max_vel,
        const double& min_vel,
        const double& max_acc,
        const double& min_acc
) {
        maxVel = (max_vel); 
        minVel = (min_vel); 
        maxAcc = (max_acc); 
        minAcc = (min_acc); 
        previousVel = (0.0);
}

void rpm_limiter::limit(double& rpm, const double& dt)
{
    double tempRPM = rpm;
    // Limit Acceleration:
    const double minVelDiff = minAcc * dt;
    const double maxVelDiff = maxAcc * dt;
    const double idealAcc = std::clamp(
        tempRPM - previousVel, minVelDiff, maxVelDiff
    );
    tempRPM = previousVel + idealAcc;
    // Limit Velocity:
    tempRPM = std::clamp(tempRPM, minVel, maxVel);
    rpm = tempRPM;
    previousVel = tempRPM;
}