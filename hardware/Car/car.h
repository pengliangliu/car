
#ifndef CAR
#define CAR

#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"
void motor_forward(void);
void motor_turn_left(void);
void motor_turn_right(void);
void motor_backward(void);
void car_stright(uint16_t pwm1, uint16_t pwm2);
void car_left(uint16_t pwm1, uint16_t pwm2);
void car_right(uint16_t pwm1, uint16_t pwm2);
void car_stop(void);


#ifdef __cplusplus
}
#endif
#endif
