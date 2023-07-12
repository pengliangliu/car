#ifndef XUNJI_H
#define XUNJI_H

#ifdef __cplusplus
extern "C" {
#endif
#include  "main.h"

void readLEDsState(GPIO_PinState *ledStates);
int processLEDStates(GPIO_PinState *ledStates);


#ifdef __cplusplus
}
#endif
#endif
