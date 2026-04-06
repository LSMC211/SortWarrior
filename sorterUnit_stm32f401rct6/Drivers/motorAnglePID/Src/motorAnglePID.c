#include "motorAnglePID.h"
#include "stm32f4xx_hal.h"
#include <stdlib.h>

typedef struct {
    float setpoint;
    float errPrev;
    uint32_t prevTime;
    uint32_t prevZeroCrossingTime;
    bool prevIsZero;
} PIDstate_TypeDef;

MotorHandle_TypeDef MotorHandle;
PIDsettings_TypeDef PIDsettings;
PIDstate_TypeDef PIDstate;


uint32_t ticksInMs = 0;

void angleMotorInit(MotorHandle_TypeDef *motor, PIDsettings_TypeDef *pid) {
    PIDstate.setpoint = 0;
    PIDstate.errPrev = 0;
    PIDstate.prevTime = 0;
    PIDstate.prevZeroCrossingTime = 0;
    PIDstate.prevIsZero = false;


    PIDsettings = *pid;
    MotorHandle = *motor;

    *(MotorHandle.encoderCNT) = 32768;
    PIDstate.setpoint = 32768;
}

void motorSetAngle(float angle) {
    PIDstate.setpoint = angle * MotorHandle.ppr / 360.0f;
}

void motorAddAngle(float angle) {
    PIDstate.setpoint += angle * MotorHandle.ppr / 360.0f;
}

void setMotorSpeed(int32_t speed) {
    if (speed > 0) {
        HAL_GPIO_WritePin(MotorHandle.dirPort, MotorHandle.dirPinA, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MotorHandle.dirPort, MotorHandle.dirPinB, GPIO_PIN_RESET);
    } 
    else if (speed < 0) {
        HAL_GPIO_WritePin(MotorHandle.dirPort, MotorHandle.dirPinA, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MotorHandle.dirPort, MotorHandle.dirPinB, GPIO_PIN_SET);
    }
    else {
        HAL_GPIO_WritePin(MotorHandle.dirPort, MotorHandle.dirPinA, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MotorHandle.dirPort, MotorHandle.dirPinB, GPIO_PIN_SET);
    }

    speed = abs(speed);
    if(speed<MotorHandle.minPwmValue) {
        speed = MotorHandle.minPwmValue;
    }
    else if(speed>MotorHandle.maxPwmValue) {
        speed = MotorHandle.maxPwmValue;
    }

    *(MotorHandle.pwmCCR) = speed;
}

float dt = 0;
float encoderVal = 0;
float motorP = 0;
float motorI = 0;
float motorD = 0;
float motorPIDoutput = 0;


bool angleMotorTick() {
    dt = (MotorHandle.timeTimer->CNT - PIDstate.prevTime);

    encoderVal = (float)(*MotorHandle.encoderCNT);

    motorP = PIDstate.setpoint - encoderVal;
    motorI += motorP * dt;
    motorD = (motorP - PIDstate.errPrev) / dt;

    motorPIDoutput = PIDsettings.Kp * motorP + PIDsettings.Ki * motorI + PIDsettings.Kd * motorD;
    if(motorPIDoutput>65535) motorPIDoutput = 65535;
    else if(motorPIDoutput<-65535) motorPIDoutput = -65535;

    if(motorPIDoutput==0) {
        if(!PIDstate.prevIsZero) {
            PIDstate.prevZeroCrossingTime = HAL_GetTick();
        }
        PIDstate.prevIsZero = true;
    }
    else {
        PIDstate.prevIsZero = false;
    }

    setMotorSpeed(motorPIDoutput);

    PIDstate.errPrev = motorP;
    PIDstate.prevTime = MotorHandle.timeTimer->CNT;
    
    if(HAL_GetTick() - PIDstate.prevZeroCrossingTime >= PIDsettings.msZeroTimeout && PIDstate.prevZeroCrossingTime!=0 && PIDstate.prevIsZero) {
        return true;
    }
    else {
        return false;
    }
}