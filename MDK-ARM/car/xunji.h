#ifndef XUNJI_H
#define XUNJI_H

#ifdef __cplusplus
extern "C" {
#endif
#include  "main.h"
#include  "pid.h"

int readLEDsState(GPIO_PinState *ledStates);
int processLEDStates(GPIO_PinState *ledStates);
void track(int flag,int speed);
void track_pid(int target_left,int target_right) ;

 extern  PID_Controller pid_track_left;
 extern  PID_Controller pid_track_right;
#ifdef __cplusplus
}
#endif
#endif
