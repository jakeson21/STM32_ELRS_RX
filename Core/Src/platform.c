/*
 * platform.c
 *
 *  Created on: Nov 11, 2023
 *      Author: Fuguru
 */

#include "platform.h"

static int32_t AltitudeCm = 0;
static uint16_t BatteryAverageCellVoltage = 0;
static uint16_t LegacyBatteryVoltage = 0;
static int32_t Amperage = 0;
static int32_t MAhDrawn = 0;
static uint8_t BatteryPercentageRemaining = 0;

// Stub out unneeded functions
int32_t getEstimatedAltitudeCm(void) { return AltitudeCm++; }
uint16_t getBatteryAverageCellVoltage(void) { return BatteryAverageCellVoltage++; }
uint16_t getLegacyBatteryVoltage(void) { return LegacyBatteryVoltage++; }
int32_t getAmperage(void) { return Amperage++; } // hhf
int32_t getMAhDrawn(void) { return MAhDrawn++; }
uint8_t calculateBatteryPercentageRemaining(void) { return BatteryPercentageRemaining++; }

bool airmodeIsEnabled(void) { return true; }
