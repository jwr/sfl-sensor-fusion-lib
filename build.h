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

#ifndef BUILD_H
#define BUILD_H

// coordinate system for the build
#define NED 0                       // identifier for NED (Aerospace) axes and angles
#define ANDROID 1                   // identifier for Android axes and angles
#define WIN8 2						// identifier for Windows 8 axes and angles
#define THISCOORDSYSTEM NED			// the coordinate system to be used

// normally all enabled: degrees of freedom algorithms to be executed
#define COMPUTE_1DOF_P_BASIC		// 1DOF pressure (altitude) and temperature: (pressure)
#define COMPUTE_3DOF_G_BASIC		// 3DOF accel tilt: (accel)
#define COMPUTE_3DOF_B_BASIC		// 3DOF mag eCompass (vehicle): (mag)
#define COMPUTE_3DOF_Y_BASIC		// 3DOF gyro integration: (gyro)
#define COMPUTE_6DOF_GB_BASIC		// 6DOF accel and mag eCompass: (accel + mag)
#define COMPUTE_6DOF_GY_KALMAN		// 6DOF accel and gyro (Kalman): (accel + gyro)
#define COMPUTE_9DOF_GBY_KALMAN		// 9DOF accel, mag and gyro (Kalman): (accel + mag + gyro)

// sampling rate and kalman filter timing eg (25, 1), (200, 8), (400, 16), (500, 20), (600, 24), (800, 32)
// the MULTI-(B) 9-AXIS and AGM01 boards are able to sample the gyro sensor at 800Hz with Kalman filter rates depending
// on the processor speed and number of algorithms executing in parallel.
#define SENSORFS 			200         // int32: frequency (Hz) of gyro sensor sampling process
#define OVERSAMPLE_RATIO 	8			// int32: accel and mag sampling and algorithms run at SENSORFS / OVERSAMPLE_RATIO Hz

#endif // BUILD_H
