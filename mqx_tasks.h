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
#ifndef MQX_TASKS_H
#define MQX_TASKS_H

#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "Cpu.h"
#include "MQX1.h"
#include "SystemTimer1.h"
#include "LED_RED.h"
#include "LED_GREEN.h"
#include "FTM.h"
#include "UART_A.h"
#include "I2C.h"
#include "UART_B.h"
#include "FTM.h"
#include "UART_A.h"
#include "lwevent.h"
#include "build.h"

// globals defined in mqx_tasks.c
extern struct ProjectGlobals globals;
extern uint8 sUARTOutputBuffer[];
extern uint8 sUART_A_InputBuffer[];
extern uint8 sUART_B_InputBuffer[];
extern struct PressureSensor thisPressure;   
extern struct AccelSensor thisAccel;   
extern struct MagSensor thisMag;       
extern struct GyroSensor thisGyro;     
extern struct MagCalibration thisMagCal;
extern struct MagneticBuffer thisMagBuffer;
extern struct SV_1DOF_P_BASIC thisSV_1DOF_P_BASIC;
extern struct SV_3DOF_G_BASIC thisSV_3DOF_G_BASIC;
extern struct SV_3DOF_B_BASIC thisSV_3DOF_B_BASIC;
extern struct SV_3DOF_Y_BASIC thisSV_3DOF_Y_BASIC;
extern struct SV_6DOF_GB_BASIC thisSV_6DOF_GB_BASIC;
extern struct SV_6DOF_GY_KALMAN thisSV_6DOF_GY_KALMAN;
extern struct SV_9DOF_GBY_KALMAN thisSV_9DOF_GBY_KALMAN;

// function prototypes for functions in mqx_tasks.c
void RdSensData_task(uint32_t task_init_data);
void Fusion_task(uint32_t task_init_data);
void MagCal_task(uint32_t task_init_data);

#endif // MQX_TASKS_H
