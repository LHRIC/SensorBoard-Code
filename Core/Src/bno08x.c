/**
 * @file bno08x.c
 * @brief BNO08x API implementation.
 */

#include <stdio.h>
#include <string.h>

#include "bno08x.h"

#include "sh2.h"
#include "sh2_util.h"
#include "sh2_err.h"
#include "sh2_SensorValue.h"
#include "sh2_hal_init.h"

// #define FIX_Q(n, x) ((int32_t)(x * (float)(1 << n)))

sh2_ProductIds_t prodIds;
sh2_Hal_t *pSh2Hal = 0;
bool resetOccurred = false;

// Handle non-sensor events from the sensor hub
static void eventHandler(void * cookie, sh2_AsyncEvent_t *pEvent)
{
    // If we see a reset, set a flag so that sensors will be reconfigured.
    if (pEvent->eventId == SH2_RESET) {
        resetOccurred = true;
    }
    else if (pEvent->eventId == SH2_SHTP_EVENT) {
        // printf("EventHandler  id:SHTP, %d\n", pEvent->shtpEvent);
    }
    else if (pEvent->eventId == SH2_GET_FEATURE_RESP) {
        // printf("EventHandler Sensor Config, %d\n", pEvent->sh2SensorConfigResp.sensorId);
    }
    else {
        // printf("EventHandler, unknown event Id: %d\n", pEvent->eventId);
    }
}

// // Handle sensor events.
// static void sensorHandler(void *cookie, sh2_SensorEvent_t *pEvent)
// {
// #ifdef DSF_OUTPUT
//     printDsf(pEvent);
// #else
//     printEvent(pEvent);
// #endif
// }

void init_bno08x(sh2_SensorCallback_t *callback)
{
  int status;

  // Create an instance of the SH2 interface
  pSh2Hal = sh2_hal_init();

  // Open the SH2 interface
  status = sh2_open(pSh2Hal, eventHandler, NULL);
  if (status != SH2_OK) {
      // printf("sh2_open failed, status = %d\n", status);
  }

  // Register sensor listener
  sh2_setSensorCallback(callback, NULL);
}

void service_bno08x(void)
{
  uint32_t now = pSh2Hal->getTimeUs(pSh2Hal);

  if (resetOccurred) {
    resetOccurred = false;
  }

  sh2_service();
}