#ifndef INC_GNSS_H_
#define INC_GNSS_H_

#include "main.h"
#include <stdbool.h>
#include <time.h>

typedef struct {
  I2C_HandleTypeDef *hi2c;

  uint8_t uniqueID[4];
  uint8_t uartWorkingBuffer[128];
  uint16_t uartWorkingBufferSize;

  unsigned short year;
  uint8_t yearBytes[2];
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  int32_t nano;
  bool valid;
  uint8_t fixType;
  uint8_t numSV;
  uint8_t satCount;
  struct timespec time;
  bool comm_ok;

  signed long lon;
  uint8_t lonBytes[4];
  signed long lat;
  uint8_t latBytes[4];
  float fLon;
  float fLat;

  signed long height;
  signed long hMSL;
  uint8_t hMSLBytes[4];
  unsigned long hAcc;
  unsigned long vAcc;

  signed long gSpeed;
  uint8_t gSpeedBytes[4];
  signed long headMot;
} GNSS_StateHandle;

void GNSS_Init(GNSS_StateHandle *GNSS, I2C_HandleTypeDef *hi2c);
void GNSS_GetPVTData(GNSS_StateHandle *GNSS);
void GNSS_ParseBuffer(GNSS_StateHandle *GNSS);
void GNSS_ParsePVTData(GNSS_StateHandle *GNSS);

#endif /* INC_GNSS_H_ */
