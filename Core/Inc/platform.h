/*
 * platform.h
 *
 *  Created on: Nov 10, 2023
 *      Author: Fuguru
 */

#ifndef INC_PLATFORM_H_
#define INC_PLATFORM_H_

#include <stdbool.h>
#include <stdint.h>

//#define USE_TELEMETRY
#define USE_CRSF_V3
#define USE_TELEMETRY_CRSF
#define USE_RX_RSNR
#undef USE_CRSF_CMS_TELEMETRY
#undef USE_RX_RSSI_DBM
#undef USE_MSP_OVER_TELEMETRY
#define USE_RX_EXPRESSLRS
#define USE_GPS

#define CRSF_TELEMETRY_FRAME_INTERVAL_MAX_US 20000 // 20ms

#define FC_FIRMWARE_NAME "JWM"
#define FC_VERSION_STRING "V1.0"

// Stub out unneeded functions
int32_t getEstimatedAltitudeCm(void);
uint16_t getBatteryAverageCellVoltage(void);
uint16_t getLegacyBatteryVoltage(void);
int32_t getAmperage(void);
int32_t getMAhDrawn(void);
uint8_t calculateBatteryPercentageRemaining(void);

bool airmodeIsEnabled(void);

typedef enum {
    SENSOR_GYRO = 1 << 0, // always present
    SENSOR_ACC = 1 << 1,
    SENSOR_BARO = 1 << 2,
    SENSOR_MAG = 1 << 3,
    SENSOR_SONAR = 1 << 4,
    SENSOR_RANGEFINDER = 1 << 4,
    SENSOR_GPS = 1 << 5,
    SENSOR_GPSMAG = 1 << 6
} sensors_e;

extern uint32_t microsISR(void);
extern uint32_t micros(void);
extern uint32_t millis(void);

#endif /* INC_PLATFORM_H_ */
