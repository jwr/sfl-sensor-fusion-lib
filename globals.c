// Copyright (c) 2014, 2015, Freescale Semiconductor, Inc.

// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Freescale Semiconductor, Inc. nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL FREESCALE SEMICONDUCTOR, INC. BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "globals.h"
#include "types.h"
#include "fusion.h"
#include "magnetic.h"

uint32_t loopcounter = 0;

// sensor data structures
#ifdef PRESSURE_SENSOR
struct PressureSensor thisPressure;		// this pressure sensor
#endif

#ifdef ACCEL_SENSOR
struct AccelSensor thisAccel;			// this accelerometer
#endif

#ifdef MAG_SENSOR
struct MagSensor thisMag;				// this magnetometer
struct MagCalibration thisMagCal;		// hard and soft iron magnetic calibration
struct MagneticBuffer thisMagBuffer;	// magnetometer measurement buffer
#endif

#ifdef GYRO_SENSOR
struct GyroSensor thisGyro;				// this gyro
#endif

// 1DOF pressure structure
#if defined COMPUTE_1DOF_P_BASIC
struct SV_1DOF_P_BASIC thisSV_1DOF_P_BASIC;					
#endif
// 3DOF accelerometer (Basic) structure
#if defined COMPUTE_3DOF_G_BASIC
struct SV_3DOF_G_BASIC thisSV_3DOF_G_BASIC;
#endif
// 3DOF magnetometer (Basic) structure
#if defined COMPUTE_3DOF_B_BASIC
struct SV_3DOF_B_BASIC thisSV_3DOF_B_BASIC;					
#endif
// 3DOF gyro (Basic) structure
#if defined COMPUTE_3DOF_Y_BASIC
struct SV_3DOF_Y_BASIC thisSV_3DOF_Y_BASIC;					
#endif
// 6DOF accelerometer and magnetometer (Basic) structure
#if defined COMPUTE_6DOF_GB_BASIC
struct SV_6DOF_GB_BASIC thisSV_6DOF_GB_BASIC;					
#endif
// 6DOF accelerometer and gyro (Kalman) structure
#if defined COMPUTE_6DOF_GY_KALMAN
struct SV_6DOF_GY_KALMAN thisSV_6DOF_GY_KALMAN;
#endif
// 9DOF accelerometer, magnetometer and gyro (Kalman) structure
#if defined COMPUTE_9DOF_GBY_KALMAN
struct SV_9DOF_GBY_KALMAN thisSV_9DOF_GBY_KALMAN;
#endif
