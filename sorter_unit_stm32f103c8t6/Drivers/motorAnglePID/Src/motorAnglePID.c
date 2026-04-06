#include "motorAnglePID.h"
#include "stm32f103xb.h"
#include <stdlib.h>

typedef struct {
    float setpoint;
    float errPrev;
    float prevTime;
    float prevZeroCrossingTime;
} PIDstate_TypeDef;

MotorHandle_TypeDef MotorHandle;
PIDsettings_TypeDef PIDsettings;
PIDstate_TypeDef PIDstate;


uint32_t ticksInMs = 0;

void angleMotorInit(MotorHandle_TypeDef *motor, PIDsettings_TypeDef *pid) {
    PIDstate.setpoint = 0;
    PIDstate.errPrev = 0;
    PIDstate.prevTime = 0;

    PIDsettings = *pid;
    MotorHandle = *motor;
    ticksInMs = (SystemCoreClock / (MotorHandle.timeTimer->PSC + 1)) / 1000;
}

void motorSetAngle(float angle) {
    PIDstate.setpoint = angle * MotorHandle.ppr / 360.0f;
}

void setMotorSpeed(int16_t speed) {
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
    if(speed<MotorHandle.minPwmValue) speed = MotorHandle.minPwmValue;
    if(speed>MotorHandle.maxPwmValue) speed = MotorHandle.maxPwmValue;

    *(MotorHandle.pwmCCR) = speed;
}

float dt = 0;
float encoderVal = 0;
float motorP = 0;
float motorI = 0;
float motorD = 0;
float motorPIDoutput = 0;


bool tick() {
    dt = (MotorHandle.timeTimer->CNT - PIDstate.prevTime);

    encoderVal = (float)(*MotorHandle.encoderCNT);

    motorP = PIDstate.setpoint - encoderVal;
    motorI += motorP * dt;
    motorD = (motorP - PIDstate.errPrev) / dt;

    motorPIDoutput = PIDsettings.Kp * motorP + PIDsettings.Ki * motorI + PIDsettings.Kd * motorD;
    if(motorPIDoutput>65535) motorPIDoutput = 65535;
    else if(motorPIDoutput<-65535) motorPIDoutput = -65535;
    else if(motorPIDoutput==0) PIDstate.prevZeroCrossingTime = MotorHandle.timeTimer->CNT;

    setMotorSpeed(motorPIDoutput);

    PIDstate.errPrev = motorP;
    PIDstate.prevTime = MotorHandle.timeTimer->CNT;

    if(MotorHandle.timeTimer->CNT - PIDstate.prevZeroCrossingTime < ticksInMs * PIDsettings.msZeroTimeout) {
        return true;
    }
    else {
        return false;
    }
}