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
// This is a Processor-Expert generated file which is then updated with custom
// code on an application-specific basic.  It contains various functions invoked
// from hardware interrupt handlers.  This also includes the UART_A input command
// interpreter.
//
#include "Cpu.h"
#include "Events.h"
#include "mqx_tasks.h"

/* User includes (#include below this line is not maintained by Processor Expert) */
#include "build.h"
#include "types.h"
#include "fusion.h"
#include "magnetic.h"

void Cpu_OnNMIINT(void)
{
	// this interupt is not used but note that some Kinetis Freedom boards do not have pullups on NMI allowing
	// the NMI pin to float but this interrupt should never be called since it's disabled in Processor Expert.
	
	return;
}

void FTM_OnCounterRestart(LDD_TUserData *UserDataPtr)
{
	// this function is called (typically at 200Hz) by the hardware clock interrupt and drives the sensor
	// read function and indirectly the fusion and magnetic calibration functions which
	// are software locked to the sensor read process
	
	// enable the (typically 200Hz) sensor read event
	_lwevent_set(&(globals.SamplingEventStruct), 1);

	return;
}

void I2C_OnMasterBlockSent(LDD_TUserData *UserDataPtr)
{
	// return with no action
	return;
}

void I2C_OnMasterBlockReceived(LDD_TUserData *UserDataPtr)
{
	// return with no action
	return;
}

void I2C_OnError(LDD_TUserData *UserDataPtr)
{
	// return with no action
	return;
}

// UART_A goes to shield and UART_B to OpenSDA to support wireless and wired USB connectivity
void UART_A_OnBlockSent(LDD_TUserData *UserDataPtr)
{
	// return with no action
	return;
}

// UART_A goes to shield and UART_B to OpenSDA to support wireless and wired USB connectivity
void UART_B_OnBlockSent(LDD_TUserData *UserDataPtr)
{
	// return with no action
	return;
}

// UART_A goes to shield and UART_B to OpenSDA to support wireless and wired USB connectivity
void UART_A_OnTxComplete(LDD_TUserData *UserDataPtr)
{
	// return with no action
	return;
}

// UART_A goes to shield and UART_B to OpenSDA to support wireless and wired USB connectivity
void UART_B_OnTxComplete(LDD_TUserData *UserDataPtr)
{
	// return with no action
	return;
}

void DecodeCommandBytes(char iCommandBuffer[], uint8 sUART_InputBuffer[], uint16 nbytes)
{
	int32 isum;							// 32 bit command identifier
	int16 i, j;							// loop counters
	
	// parse all received bytes in sUARTInputBuf into the iCommandBuffer delay line
	for (i = 0; i < nbytes; i++)
	{
		// shuffle the iCommandBuffer delay line and add the new command byte
		for (j = 0; j < 3; j++)
			iCommandBuffer[j] = iCommandBuffer[j + 1];
		iCommandBuffer[3] = sUART_InputBuffer[i];
		
		// check if we have a valid command yet
		isum = ((((((int32)iCommandBuffer[0] << 8) + iCommandBuffer[1]) << 8) + iCommandBuffer[2]) << 8) + iCommandBuffer[3];
		switch (isum)
		{
		// "VG+ " = enable angular velocity packet transmission
		case ((((('V' << 8) + 'G') << 8) + '+') << 8) + ' ':
			globals.AngularVelocityPacketOn = true;
			iCommandBuffer[3] = '~';
		break;
		// "VG- " = disable angular velocity packet transmission
		case ((((('V' << 8) + 'G') << 8) + '-') << 8) + ' ':
			globals.AngularVelocityPacketOn = false; 
			iCommandBuffer[3] = '~';
		break;
		
		// "DB+ " = enable debug packet transmission
		case ((((('D' << 8) + 'B') << 8) + '+') << 8) + ' ':
			globals.DebugPacketOn = true;
			iCommandBuffer[3] = '~';
		break;
		// "DB- " = disable debug packet transmission
		case ((((('D' << 8) + 'B') << 8) + '-') << 8) + ' ':
			globals.DebugPacketOn = false; 
			iCommandBuffer[3] = '~';
		break;
		
		// "Q3  " = transmit 3-axis accelerometer quaternion in standard packet
		case ((((('Q' << 8) + '3') << 8) + ' ') << 8) + ' ':
	#if defined COMPUTE_3DOF_G_BASIC
			globals.QuaternionPacketType = Q3;
	#endif
		iCommandBuffer[3] = '~';
		break;
		// "Q3M " = transmit 3-axis magnetometer quaternion in standard packet
		case ((((('Q' << 8) + '3') << 8) + 'M') << 8) + ' ':
	#if defined COMPUTE_3DOF_B_BASIC
			globals.QuaternionPacketType = Q3M;
	#endif
		iCommandBuffer[3] = '~';
		break;
		// "Q3G " = transmit 3-axis gyro quaternion in standard packet
		case ((((('Q' << 8) + '3') << 8) + 'G') << 8) + ' ':
	#if defined COMPUTE_3DOF_Y_BASIC
			globals.QuaternionPacketType = Q3G; 
	#endif
		iCommandBuffer[3] = '~';
		break;
		// "Q6MA" = transmit 6-axis mag/accel quaternion in standard packet
		case ((((('Q' << 8) + '6') << 8) + 'M') << 8) + 'A':
	#if defined COMPUTE_6DOF_GB_BASIC
			globals.QuaternionPacketType = Q6MA;
	#endif
		iCommandBuffer[3] = '~';
		break;	
		// "Q6AG" = transmit 6-axis accel/gyro quaternion in standard packet
		case ((((('Q' << 8) + '6') << 8) + 'A') << 8) + 'G':
	#if defined COMPUTE_6DOF_GY_KALMAN
		globals.QuaternionPacketType = Q6AG;
	#endif
		iCommandBuffer[3] = '~';
		break;
		// "Q9  " = transmit 9-axis quaternion in standard packet (default)
		case ((((('Q' << 8) + '9') << 8) + ' ') << 8) + ' ':
#if defined COMPUTE_9DOF_GBY_KALMAN
		globals.QuaternionPacketType = Q9;
#endif
		iCommandBuffer[3] = '~';
		break;
		
		// "RPC+" = Roll/Pitch/Compass on
		case ((((('R' << 8) + 'P') << 8) + 'C') << 8) + '+':
			globals.RPCPacketOn = true; 
			iCommandBuffer[3] = '~';
		break;
		// "RPC-" = Roll/Pitch/Compass off
		case ((((('R' << 8) + 'P') << 8) + 'C') << 8) + '-':
			globals.RPCPacketOn = false; 
			iCommandBuffer[3] = '~';
		break;
		
		// "ALT+" = Altitude packet on
		case ((((('A' << 8) + 'L') << 8) + 'T') << 8) + '+':
			globals.AltPacketOn = true; 
			iCommandBuffer[3] = '~';
		break;
		// "ALT-" = Altitude packet off
		case ((((('A' << 8) + 'L') << 8) + 'T') << 8) + '-':
			globals.AltPacketOn = false; 
			iCommandBuffer[3] = '~';
		break;

		// "RST " = Soft reset
		case ((((('R' << 8) + 'S') << 8) + 'T') << 8) + ' ':	
			// reset sensor fusion
			fInitFusion();
			// reset magnetic calibration and magnetometer data buffer
	#if defined COMPUTE_3DOF_B_BASIC || defined COMPUTE_6DOF_GB_BASIC || defined COMPUTE_9DOF_GBY_KALMAN
			fInitMagCalibration(&thisMagCal, &thisMagBuffer);
	#endif
			iCommandBuffer[3] = '~';
		break;
		
		// "RINS" = Reset INS inertial navigation velocity and position
		case ((((('R' << 8) + 'I') << 8) + 'N') << 8) + 'S':
#ifdef COMPUTE_9DOF_GBY_KALMAN
		for (i = CHX; i <= CHZ; i++)
		{
			thisSV_9DOF_GBY_KALMAN.fVelGl[i] = 0.0F;
			thisSV_9DOF_GBY_KALMAN.fDisGl[i] = 0.0F;
		}
#endif
		iCommandBuffer[3] = '~';
		break;

		default:
			// no action
			break;
		}	
	} // end of loop over received characters
	
	return;
}

// UART_A_OnBlockReceived and UART_B_OnBlockReceived function are called when one or more characters have arrived 
// in the UART's receive buffer
// incoming characters are placed in a delay line and processed whenever a valid command is received.
// note also that although this callback is theoretically called whenever a single byte is received, 
// in practice there may be bursts of more than one byte in the receive buffer.
// all received bytes are processed before this callback is completed.
// the command is cleared by over-writing the last byte [3] of the 4 byte command with ~ which is never present
// in any issued command.

// this function decodes instructions received over UART_A (Bluetooth module on shield)
void UART_A_OnBlockReceived(LDD_TUserData *UserDataPtr)
{
	uint16 nbytes;								// number of bytes received
	static char iCommandBuffer_A[5] = "~~~~";	// 5 bytes long to include the unused terminating \0

	// determine how many bytes are available in the UART_A receive buffer and decode
	nbytes = UART_A_GetReceivedDataNum(UART_A_DeviceData);
	DecodeCommandBytes(iCommandBuffer_A, sUART_A_InputBuffer, nbytes);

	// generate the next callback event to this function when the next character arrives
	// this function is non-blocking
	UART_A_ReceiveBlock(UART_A_DeviceData, sUART_A_InputBuffer, 1);

	return;
}

// this function decodes instructions received over UART_B (OpenSDA and USB)
void UART_B_OnBlockReceived(LDD_TUserData *UserDataPtr)
{
	uint16 nbytes;								// number of bytes received
	static char iCommandBuffer_B[5] = "~~~~";	// 5 bytes long to include the unused terminating \0

	// determine how many bytes are available in the UART_B receive buffer and decode
	nbytes = UART_B_GetReceivedDataNum(UART_B_DeviceData);
	DecodeCommandBytes(iCommandBuffer_B, sUART_B_InputBuffer, nbytes);

	// generate the next callback event to this function when the next character arrives
	// this function is non-blocking
	UART_B_ReceiveBlock(UART_B_DeviceData, sUART_B_InputBuffer, 1);

	return;
}

