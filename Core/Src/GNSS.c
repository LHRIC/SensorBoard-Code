#include "GNSS.h"

#include <stdbool.h>
#include <string.h>

#define GNSS_I2C_DEVICE_ADDRESS (0x42 << 1)
#define GNSS_I2C_BYTES_AVAILABLE_HIGH 0xFD
#define GNSS_I2C_DATA_STREAM 0xFF

#define GNSS_PVT_MESSAGE_SIZE 100
#define GNSS_I2C_TIMEOUT_MS 1000
#define GNSS_WAIT_TIMEOUT_MS 200

static uint16_t read_u16_le(const uint8_t *data) {
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint16_t read_u16_be(const uint8_t *data) {
  return ((uint16_t)data[0] << 8) | (uint16_t)data[1];
}

static uint32_t read_u32_le(const uint8_t *data) {
  return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static int32_t read_i32_le(const uint8_t *data) {
  return (int32_t)read_u32_le(data);
}

static bool is_leap_year(unsigned year_offset) {
  unsigned year = year_offset + 1900U;
  return (year % 4U) == 0U && ((year % 100U) != 0U || (year % 400U) == 0U);
}

static time_t gnss_timegm(struct tm *tm) {
  static const unsigned ndays[2][12] = {
      {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},
      {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}};

  time_t res = 0;
  int year;

  for (year = 70; year < tm->tm_year; ++year) {
    res += is_leap_year((unsigned)year) ? 366 : 365;
  }

  for (int month = 0; month < tm->tm_mon; ++month) {
    res += ndays[is_leap_year((unsigned)tm->tm_year)][month];
  }

  res += tm->tm_mday - 1;
  res *= 24;
  res += tm->tm_hour;
  res *= 60;
  res += tm->tm_min;
  res *= 60;
  res += tm->tm_sec;
  return res;
}

static void GNSS_UpdateTime(GNSS_StateHandle *GNSS) {
  struct tm tm = {0};
  tm.tm_year = (int)GNSS->year - 1900;
  tm.tm_mon = (int)GNSS->month - 1;
  tm.tm_mday = (int)GNSS->day;
  tm.tm_hour = (int)GNSS->hour;
  tm.tm_min = (int)GNSS->min;
  tm.tm_sec = (int)GNSS->sec;

  time_t seconds = gnss_timegm(&tm);
  int64_t nanoseconds = GNSS->nano;

  if (nanoseconds < 0) {
    seconds -= 1;
    nanoseconds += 1000000000LL;
  } else if (nanoseconds >= 1000000000LL) {
    seconds += (time_t)(nanoseconds / 1000000000LL);
    nanoseconds %= 1000000000LL;
  }

  GNSS->yearBytes[0] = (uint8_t)(GNSS->year & 0xFF);
  GNSS->yearBytes[1] = (uint8_t)((GNSS->year >> 8) & 0xFF);

  GNSS->time.tv_sec = seconds;
  GNSS->time.tv_nsec = (long)nanoseconds;
}

static uint16_t GNSS_BytesAvailable(GNSS_StateHandle *GNSS) {
  uint8_t data[2] = {0};
  if (HAL_I2C_Mem_Read(GNSS->hi2c, GNSS_I2C_DEVICE_ADDRESS,
                       GNSS_I2C_BYTES_AVAILABLE_HIGH, I2C_MEMADD_SIZE_8BIT,
                       data, sizeof(data), GNSS_I2C_TIMEOUT_MS) != HAL_OK) {
    GNSS->comm_ok = false;
    return 0;
  }

  GNSS->comm_ok = true;
  return read_u16_be(data);
}

void GNSS_Init(GNSS_StateHandle *GNSS, I2C_HandleTypeDef *hi2c) {
  memset(GNSS, 0, sizeof(*GNSS));
  GNSS->hi2c = hi2c;
  GNSS->comm_ok =
      (HAL_I2C_IsDeviceReady(GNSS->hi2c, GNSS_I2C_DEVICE_ADDRESS, 3, 100) ==
       HAL_OK);
  HAL_Delay(300);
}

void GNSS_GetPVTData(GNSS_StateHandle *GNSS) {
  static const uint8_t getPVTData[] = {0xB5, 0x62, 0x01, 0x07,
                                       0x00, 0x00, 0x08, 0x19};

  if (HAL_I2C_Mem_Write(GNSS->hi2c, GNSS_I2C_DEVICE_ADDRESS,
                        GNSS_I2C_DATA_STREAM, I2C_MEMADD_SIZE_8BIT,
                        (uint8_t *)getPVTData, sizeof(getPVTData),
                        GNSS_I2C_TIMEOUT_MS) != HAL_OK) {
    GNSS->comm_ok = false;
    GNSS->uartWorkingBufferSize = 0;
    return;
  }

  GNSS->comm_ok = true;

  uint32_t start = HAL_GetTick();
  uint16_t bytes = GNSS_BytesAvailable(GNSS);
  while (bytes < GNSS_PVT_MESSAGE_SIZE &&
         (HAL_GetTick() - start) < GNSS_WAIT_TIMEOUT_MS) {
    HAL_Delay(1);
    bytes = GNSS_BytesAvailable(GNSS);
  }

  if (bytes > sizeof(GNSS->uartWorkingBuffer)) {
    bytes = sizeof(GNSS->uartWorkingBuffer);
  }

  if (bytes == 0) {
    GNSS->uartWorkingBufferSize = 0;
    return;
  }

  if (HAL_I2C_Mem_Read(GNSS->hi2c, GNSS_I2C_DEVICE_ADDRESS,
                       GNSS_I2C_DATA_STREAM, I2C_MEMADD_SIZE_8BIT,
                       GNSS->uartWorkingBuffer, bytes,
                       GNSS_I2C_TIMEOUT_MS) != HAL_OK) {
    GNSS->comm_ok = false;
    GNSS->uartWorkingBufferSize = 0;
    return;
  }

  GNSS->comm_ok = true;
  GNSS->uartWorkingBufferSize = bytes;
}

void GNSS_ParseBuffer(GNSS_StateHandle *GNSS) {
  const uint8_t *buffer = GNSS->uartWorkingBuffer;
  size_t size = GNSS->uartWorkingBufferSize;

  for (size_t index = 0; index + 8U <= size; ++index) {
    if (buffer[index] != 0xB5 || buffer[index + 1] != 0x62) {
      continue;
    }

    uint8_t message_class = buffer[index + 2];
    uint8_t message_id = buffer[index + 3];
    uint16_t payload_size = read_u16_le(&buffer[index + 4]);
    size_t frame_size = (size_t)payload_size + 8U;

    if (index + frame_size > size || message_class != 0x01 ||
        message_id != 0x07 || payload_size < 92U) {
      continue;
    }

    uint8_t checksum_a = 0;
    uint8_t checksum_b = 0;
    for (size_t i = 2; i < frame_size - 2U; ++i) {
      checksum_a = (uint8_t)(checksum_a + buffer[index + i]);
      checksum_b = (uint8_t)(checksum_b + checksum_a);
    }

    if (checksum_a != buffer[index + frame_size - 2U] ||
        checksum_b != buffer[index + frame_size - 1U]) {
      continue;
    }

    memmove(GNSS->uartWorkingBuffer, &buffer[index], frame_size);
    GNSS->uartWorkingBufferSize = (uint16_t)frame_size;
    GNSS_ParsePVTData(GNSS);
    return;
  }
}

void GNSS_ParsePVTData(GNSS_StateHandle *GNSS) {
  const uint8_t *frame = GNSS->uartWorkingBuffer;

  GNSS->year = read_u16_le(&frame[10]);
  GNSS->month = frame[12];
  GNSS->day = frame[13];
  GNSS->hour = frame[14];
  GNSS->min = frame[15];
  GNSS->sec = frame[16];
  GNSS->nano = read_i32_le(&frame[22]);
  GNSS->fixType = frame[26];
  GNSS->numSV = frame[29];
  GNSS->satCount = GNSS->numSV;

  GNSS->lon = read_i32_le(&frame[30]);
  GNSS->lat = read_i32_le(&frame[34]);
  GNSS->height = read_i32_le(&frame[38]);
  GNSS->hMSL = read_i32_le(&frame[42]);
  GNSS->hAcc = read_u32_le(&frame[46]);
  GNSS->vAcc = read_u32_le(&frame[50]);
  GNSS->gSpeed = read_i32_le(&frame[66]);
  GNSS->headMot = read_i32_le(&frame[70]);

  GNSS->lonBytes[0] = frame[30];
  GNSS->lonBytes[1] = frame[31];
  GNSS->lonBytes[2] = frame[32];
  GNSS->lonBytes[3] = frame[33];
  GNSS->latBytes[0] = frame[34];
  GNSS->latBytes[1] = frame[35];
  GNSS->latBytes[2] = frame[36];
  GNSS->latBytes[3] = frame[37];
  GNSS->hMSLBytes[0] = frame[42];
  GNSS->hMSLBytes[1] = frame[43];
  GNSS->hMSLBytes[2] = frame[44];
  GNSS->hMSLBytes[3] = frame[45];
  GNSS->gSpeedBytes[0] = frame[66];
  GNSS->gSpeedBytes[1] = frame[67];
  GNSS->gSpeedBytes[2] = frame[68];
  GNSS->gSpeedBytes[3] = frame[69];

  GNSS->fLon = (float)GNSS->lon / 10000000.0f;
  GNSS->fLat = (float)GNSS->lat / 10000000.0f;
  GNSS->valid = (GNSS->fixType >= 3U);

  GNSS_UpdateTime(GNSS);
}
