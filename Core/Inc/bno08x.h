/*
* @file bno08x.h
* @brief BNO08x API.
*/

#ifndef BNO08X_H
#define BNO08X_H

#include "sh2.h"

void init_bno08x(sh2_SensorCallback_t *callback);
void service_bno08x(void);

#endif // BNO08X_H