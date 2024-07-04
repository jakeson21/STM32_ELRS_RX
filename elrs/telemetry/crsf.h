/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"
#include "rx/crsf_protocol.h"

void initCrsfTelemetry(void);
uint32_t getCrsfDesiredSpeed(void);
void setCrsfDefaultSpeed(void);
bool checkCrsfTelemetryState(void);
void handleCrsfTelemetry(timeUs_t currentTimeUs);
void crsfScheduleDeviceInfoResponse(void);
void crsfScheduleMspResponse(uint8_t requestOriginID);
void crsfHandleDeviceInfoResponse(uint8_t *payload);
int getCrsfFrame(uint8_t *frame, crsfFrameType_e frameType);
void crsfProcessCommand(uint8_t *frameStart);

#if defined(USE_CRSF_V3)
void speedNegotiationProcess(uint32_t currentTime);
bool crsfBaudNegotiationInProgress(void);
uint32_t getCrsfCachedBaudrate(void);
#endif

typedef struct gpsCoordinateDDDMMmmmm_s {
    int16_t dddmm;
    int16_t mmmm;
} gpsCoordinateDDDMMmmmm_t;

/* LLH Location in NEU axis system */
typedef struct gpsLocation_s {
    int32_t lat;                    // latitude * 1e+7
    int32_t lon;                    // longitude * 1e+7
    int32_t altCm;                  // altitude in 0.01m
} gpsLocation_t;

/* A value below 100 means great accuracy is possible with GPS satellite constellation */
typedef struct gpsDilution_s {
    uint16_t pdop;                  // positional DOP - 3D (* 100)
    uint16_t hdop;                  // horizontal DOP - 2D (* 100)
    uint16_t vdop;                  // vertical DOP   - 1D (* 100)
} gpsDilution_t;

/* Only available on U-blox protocol */
typedef struct gpsAccuracy_s {
    uint32_t hAcc;                  // horizontal accuracy in mm
    uint32_t vAcc;                  // vertical accuracy in mm
    uint32_t sAcc;                  // speed accuracy in mm/s
} gpsAccuracy_t;

typedef struct gpsSolutionData_s {
    gpsLocation_t llh;
    gpsDilution_t dop;
    gpsAccuracy_t acc;
    uint16_t speed3d;               // speed in 0.1m/s
    uint16_t groundSpeed;           // speed in 0.1m/s
    uint16_t groundCourse;          // degrees * 10
    uint8_t numSat;
    uint32_t time;                  // GPS msToW
    uint32_t navIntervalMs;         // interval between nav solutions in ms
} gpsSolutionData_t;

