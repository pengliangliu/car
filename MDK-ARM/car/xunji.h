#ifndef XUNJI_H
#define XUNJI_H

#ifdef __cplusplus
extern "C" {
#endif
#include  "main.h"

int readLEDsState(GPIO_PinState *ledStates);
int processLEDStates(GPIO_PinState *ledStates);
void track(int flag,int speed);

#ifdef __cplusplus
}
#endif
#endif
