#include "main.h"


typedef struct {
    volatile uint32_t *pwmCCR;
    GPIO_TypeDef *dirPort;
    uint16_t dirPinA;
    uint16_t dirPinB;
    volatile uint32_t *encoderCNT;
    uint16_t ppr;
    TIM_TypeDef *timeTimer;

    uint16_t minPwmValue;
    uint16_t maxPwmValue;

} MotorHandle_TypeDef;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    uint16_t msZeroTimeout;
} PIDsettings_TypeDef;

void angleMotorInit(MotorHandle_TypeDef *motor, PIDsettings_TypeDef *pid);
void setMotorSpeed(int32_t speed);
void motorSetAngle(float angle);
bool angleMotorTick();