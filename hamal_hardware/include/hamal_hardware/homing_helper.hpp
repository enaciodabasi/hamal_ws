/**
 * @file homing_helper.hpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-09-19
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef HOMING_HELPER_HPP_
#define HOMING_HELPER_HPP_

#include "hamal_hardware/hamal_hardware_defs.hpp"

struct HomingHelper
{
    bool isHomingActive;
    bool isHomingInProgress;
    bool isHomingSetupDone;
    
    int8_t homingMethod;
    int32_t switchSearchSpeed;
    int32_t zeroSearchSpeed;
    int32_t homingAccel;
    
    uint16_t previousCtrlWord;

    bool homingErrorBit = false;
    bool homingAttainedBit = false;
    bool targetReachedBit = false;

    HomingHelper()
    {
        isHomingActive = false;
        isHomingInProgress = false;
        isHomingSetupDone = false;
        switchSearchSpeed = 0;
        homingMethod = 0x11;
        zeroSearchSpeed = 0;
        homingAccel = 0;
        previousCtrlWord = 0x0;
    }

    inline const HomingStatus getCurrentHomingStatus()
    {
        if(!homingErrorBit && !homingAttainedBit && !targetReachedBit){
            return HomingStatus::HomingIsPerformed;
        }
        else if(!homingErrorBit && !homingAttainedBit && targetReachedBit){
            return HomingStatus::HomingIsInterruptedOrNotStarted;
        }
        else if(!homingErrorBit && homingAttainedBit && !targetReachedBit){
            return HomingStatus::HomingConfirmedTargetNotReached;
        }
        else if(!homingErrorBit && homingAttainedBit && targetReachedBit){
            return HomingStatus::HomingCompleted;
        }
        else if(homingErrorBit && !homingAttainedBit && !targetReachedBit){
            return HomingStatus::ErrorDetectedMotorStillRunning;
        }
        else if(homingErrorBit && !homingAttainedBit && targetReachedBit){
            return HomingStatus::ErrorDuringHomingMotorAtStandstill;
        }
    
        return HomingStatus::Unknown;
    }

    inline void reset()
    {
        isHomingActive = false;
        isHomingInProgress = false;
        isHomingSetupDone = false;
        previousCtrlWord = 0x0;
    }

};

#endif // HOMING_HELPER_HPP_