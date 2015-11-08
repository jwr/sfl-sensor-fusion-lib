#include <stdint.h>
#include <stdlib.h> // for abs()

#include "types.h"
#include "globals.h"
#include "magnetic.h"
#include "fusion.h"

void sfl_initialize() {
  int i;
  // Force a reset of all the algorithms next time they execute. The initialization will result in the default and
  // current quaternion being set to the most sophisticated algorithm supported by the build.

#if defined COMPUTE_1DOF_P_BASIC
	thisSV_1DOF_P_BASIC.resetflag = true;					
#endif
#if defined COMPUTE_3DOF_G_BASIC
	thisSV_3DOF_G_BASIC.resetflag = true;				
#endif
#if defined COMPUTE_3DOF_B_BASIC
	thisSV_3DOF_B_BASIC.resetflag = true;	
#endif
#if defined COMPUTE_3DOF_Y_BASIC
	thisSV_3DOF_Y_BASIC.resetflag = true;				
#endif
#if defined COMPUTE_6DOF_GB_BASIC
	thisSV_6DOF_GB_BASIC.resetflag = true;				
#endif
#if defined COMPUTE_6DOF_GY_KALMAN
	thisSV_6DOF_GY_KALMAN.resetflag = true;
#endif
#if defined COMPUTE_9DOF_GBY_KALMAN
	thisSV_9DOF_GBY_KALMAN.resetflag = true;
#endif

	fInitFusion();
	
	// initialize magnetometer data structure
#if defined COMPUTE_3DOF_B_BASIC || defined COMPUTE_6DOF_GB_BASIC || defined COMPUTE_9DOF_GBY_KALMAN
	// zero the calibrated measurement since this is used for indexing the magnetic buffer even before first calibration
	for (i = CHX; i <= CHZ; i++)
	  thisMag.iBcAvg[i]= 0;
#endif
  
  // initialize magnetic calibration and magnetometer data buffer
#if defined COMPUTE_3DOF_B_BASIC || defined COMPUTE_6DOF_GB_BASIC || defined COMPUTE_9DOF_GBY_KALMAN
  fInitMagCalibration(&thisMagCal, &thisMagBuffer);
#endif
}


// This function should be called at 200Hz to process the measurements. Before calling, make sure that the latest
// measurements have been read off the sensors and are stored in the appropriate structures (thisAccel, thisMag,
// thisGyro, thisPressure). The function will return 1 if sfl_fusion() should be called.
uint8_t sfl_process_measurements() {
  static int8 iCounter = 0;		// decimation counter range 0 to OVERSAMPLE_RATIO-1
  int32 iSum[3];				// array of sums
  int32 itmp;					// scratch
  int8 i, j, k, l;				// counters
  
  // read and process accelerometer sensor in every slot if present if accelerometer algorithm is in use.
#if defined COMPUTE_3DOF_G_BASIC || defined COMPUTE_6DOF_GB_BASIC || defined COMPUTE_6DOF_GY_KALMAN || defined COMPUTE_9DOF_GBY_KALMAN
  // store measurement in a buffer for later end of block processing
  for (i = CHX; i <= CHZ; i++)
	thisAccel.iGsBuffer[iCounter][i] = thisAccel.iGs[i];

  // every OVERSAMPLE_RATIO passes calculate the block averaged measurement
  if (iCounter == (OVERSAMPLE_RATIO - 1))
	{	
	  // calculate the block averaged measurement in counts and g
	  for (i = CHX; i <= CHZ; i++)
		{
		  iSum[i] = 0;
		  for (j = 0; j < OVERSAMPLE_RATIO; j++)
			iSum[i] += (int32)thisAccel.iGsBuffer[j][i];
		  // compute the average with nearest integer rounding
		  if (iSum[i] >= 0)
			thisAccel.iGsAvg[i] = (int16)((iSum[i] + (OVERSAMPLE_RATIO >> 1)) / OVERSAMPLE_RATIO);
		  else
			thisAccel.iGsAvg[i] = (int16)((iSum[i] - (OVERSAMPLE_RATIO >> 1)) / OVERSAMPLE_RATIO);
		  // convert from integer counts to float g
		  thisAccel.fGsAvg[i] = (float)thisAccel.iGsAvg[i] * thisAccel.fgPerCount;
		}
	} // end of test for end of OVERSAMPLE_RATIO block
#endif // end of check for accelerometer algorithm and sensor

  // read and process the magnetometer sensor in every slot if magnetic algorithm is in use.
#if defined COMPUTE_3DOF_B_BASIC || defined COMPUTE_6DOF_GB_BASIC || defined COMPUTE_9DOF_GBY_KALMAN
  // store in a buffer for later end of block processing
  for (i = CHX; i <= CHZ; i++)
	thisMag.iBsBuffer[iCounter][i] = thisMag.iBs[i];

  // update magnetic buffer with iBs avoiding a write to the shared structure while a calibration is in progress.
  if (!thisMagCal.iCalInProgress)
	iUpdateMagnetometerBuffer(&thisMagBuffer, &thisMag, loopcounter);

  // every OVERSAMPLE_RATIO passes calculate the block averaged and calibrated measurement using an anti-glitch filter
  // that rejects the measurement furthest from the mean. magnetometer sensors are sensitive
  // to occasional current pulses from power supply traces and so on and this is a simple method to remove these.
  if (iCounter == (OVERSAMPLE_RATIO - 1))
	{
	  // calculate the channel means using all measurements
	  for (i = CHX; i <= CHZ; i++)
		{
		  // accumulate channel sums
		  iSum[i] = 0;
		  for (j = 0; j < OVERSAMPLE_RATIO; j++)
			iSum[i] += (int32)thisMag.iBsBuffer[j][i];
		}
	  // store axis k in buffer measurement l furthest from its mean
	  itmp = 0;
	  for (i = CHX; i <= CHZ; i++)
		{
		  for (j = 0; j < OVERSAMPLE_RATIO; j++)
			{
			  if (abs((int32)thisMag.iBsBuffer[j][i] * OVERSAMPLE_RATIO - iSum[i]) >= itmp)
				{
				  k = i;
				  l = j;
				  itmp = abs((int32)thisMag.iBsBuffer[j][i] * OVERSAMPLE_RATIO - iSum[i]);
				}
			}
		}

	  // re-calculate the block averaged measurement ignoring channel k in measurement l
	  if (OVERSAMPLE_RATIO == 1)
		{
		  // use the one available measurement for averaging in this case
		  for (i = CHX; i <= CHZ; i++)
			{
			  thisMag.iBsAvg[i] = thisMag.iBsBuffer[0][i];
			}
		} // end of compute averages for OVERSAMPLE_RATIO = 1
	  else
		{
		  // sum all measurements ignoring channel k in measurement l
		  for (i = CHX; i <= CHZ; i++)
			{
			  iSum[i] = 0;
			  for (j = 0; j < OVERSAMPLE_RATIO; j++)
				{
				  if (!((i == k) && (j == l)))
					iSum[i] += (int32)thisMag.iBsBuffer[j][i];
				}
			}
		  // compute the average with nearest integer rounding
		  for (i = CHX; i <= CHZ; i++)
			{
			  if (i != k)
				{
				  // OVERSAMPLE_RATIO measurements were used
				  if (iSum[i] >= 0)
					thisMag.iBsAvg[i] = (int16)((iSum[i] + (OVERSAMPLE_RATIO >> 1)) / OVERSAMPLE_RATIO);
				  else
					thisMag.iBsAvg[i] = (int16)((iSum[i] - (OVERSAMPLE_RATIO >> 1)) / OVERSAMPLE_RATIO);
				}
			  else
				{
				  // OVERSAMPLE_RATIO - 1 measurements were used
				  if (iSum[i] >= 0)
					thisMag.iBsAvg[i] = (int16)((iSum[i] + ((OVERSAMPLE_RATIO - 1) >> 1)) / (OVERSAMPLE_RATIO - 1));
				  else
					thisMag.iBsAvg[i] = (int16)((iSum[i] - ((OVERSAMPLE_RATIO - 1) >> 1)) / (OVERSAMPLE_RATIO - 1));
				}
			}
		} // end of compute averages for OVERSAMPLE_RATIO = 1

		  // convert the averages to float
		  for (i = CHX; i <= CHZ; i++)
			thisMag.fBsAvg[i] = (float)thisMag.iBsAvg[i] * thisMag.fuTPerCount;

		  // remove hard and soft iron terms from fBsAvg (uT) to get calibrated data fBcAvg (uT), iBc (counts)
		  fInvertMagCal(&thisMag, &thisMagCal);
			
		} // end of test for end of OVERSAMPLE_RATIO block
#endif // end of check for magnetic algorithms and sensor

  // read the gyro sensor every time slot
#if defined COMPUTE_3DOF_Y_BASIC || defined COMPUTE_6DOF_GY_KALMAN || defined COMPUTE_9DOF_GBY_KALMAN
  // store in a buffer for later gyro integration by sensor fusion algorithms
  for (i = CHX; i <= CHZ; i++)
	thisGyro.iYsBuffer[iCounter][i] = thisGyro.iYs[i];
#endif // end of check for gyro algorithm and sensor

  // every OVERSAMPLE_RATIO passes zero the decimation counter and enable the sensor fusion task
  if (iCounter++ == (OVERSAMPLE_RATIO - 1)) {
	iCounter = 0;
	return 0;				// no need to run fusion yet
  } 
  return 1;					// caller should call the fusion function
}

// The fusion function should be called with the current systick value as a first parameter (CMSIS makes it available as
// SysTick->VAL) and the systick reload value as the second (SysTick->LOAD). This function will return 1 if
// sfl_magnetic_calibration() needs to be called.
uint8_t sfl_fusion(uint32_t systick_value, uint32_t systick_reload)
{
  int8 initiatemagcal = 0;		// flag to initiate a new magnetic calibration

  // 1DOF Pressure: call the low pass filter algorithm
#if defined COMPUTE_1DOF_P_BASIC
  thisSV_1DOF_P_BASIC.systick = systick_value & 0x00FFFFFF;
  fRun_1DOF_P_BASIC(&thisSV_1DOF_P_BASIC, &thisPressure);
  thisSV_1DOF_P_BASIC.systick -= systick_value & 0x00FFFFFF;
  if (thisSV_1DOF_P_BASIC.systick < 0) thisSV_1DOF_P_BASIC.systick += systick_reload;	
#endif

  // 3DOF Accel Basic: call the tilt algorithm
#if defined COMPUTE_3DOF_G_BASIC		
  thisSV_3DOF_G_BASIC.systick = systick_value & 0x00FFFFFF;		
  fRun_3DOF_G_BASIC(&thisSV_3DOF_G_BASIC, &thisAccel);
  thisSV_3DOF_G_BASIC.systick -= systick_value & 0x00FFFFFF;
  if (thisSV_3DOF_G_BASIC.systick < 0) thisSV_3DOF_G_BASIC.systick += systick_reload;
#endif

  // 3DOF Magnetometer Basic: call the 2D vehicle compass algorithm
#if defined COMPUTE_3DOF_B_BASIC
  thisSV_3DOF_B_BASIC.systick = systick_value & 0x00FFFFFF;
  fRun_3DOF_B_BASIC(&thisSV_3DOF_B_BASIC, &thisMag);
  thisSV_3DOF_B_BASIC.systick -= systick_value & 0x00FFFFFF;
  if (thisSV_3DOF_B_BASIC.systick < 0) thisSV_3DOF_B_BASIC.systick += systick_reload;	
#endif

  // 3DOF Gyro Basic: call the gyro integration algorithm
#if defined COMPUTE_3DOF_Y_BASIC	
  thisSV_3DOF_Y_BASIC.systick = systick_value & 0x00FFFFFF;
  fRun_3DOF_Y_BASIC(&thisSV_3DOF_Y_BASIC, &thisGyro);
  thisSV_3DOF_Y_BASIC.systick -= systick_value & 0x00FFFFFF;
  if (thisSV_3DOF_Y_BASIC.systick < 0) thisSV_3DOF_Y_BASIC.systick += systick_reload;
#endif

  // 6DOF Accel / Mag: Basic: call the eCompass orientation algorithm
#if defined COMPUTE_6DOF_GB_BASIC		
  thisSV_6DOF_GB_BASIC.systick = systick_value & 0x00FFFFFF;
  fRun_6DOF_GB_BASIC(&thisSV_6DOF_GB_BASIC, &thisMag, &thisAccel);
  thisSV_6DOF_GB_BASIC.systick -= systick_value & 0x00FFFFFF;
  if (thisSV_6DOF_GB_BASIC.systick < 0) thisSV_6DOF_GB_BASIC.systick += systick_reload;		
#endif

  // 6DOF Accel / Gyro: call the Kalman filter orientation algorithm
#if defined COMPUTE_6DOF_GY_KALMAN		
  thisSV_6DOF_GY_KALMAN.systick = systick_value & 0x00FFFFFF;
  fRun_6DOF_GY_KALMAN(&thisSV_6DOF_GY_KALMAN, &thisAccel, &thisGyro);
  thisSV_6DOF_GY_KALMAN.systick -= systick_value & 0x00FFFFFF;
  if (thisSV_6DOF_GY_KALMAN.systick < 0) thisSV_6DOF_GY_KALMAN.systick += systick_reload;	
#endif
  // 9DOF Accel / Mag / Gyro: call the Kalman filter orientation algorithm
#if defined COMPUTE_9DOF_GBY_KALMAN		
  thisSV_9DOF_GBY_KALMAN.systick = systick_value & 0x00FFFFFF;
  fRun_9DOF_GBY_KALMAN(&thisSV_9DOF_GBY_KALMAN, &thisAccel, &thisMag, &thisGyro, &thisMagCal);
  thisSV_9DOF_GBY_KALMAN.systick -= systick_value & 0x00FFFFFF;		
  if (thisSV_9DOF_GBY_KALMAN.systick < 0) thisSV_9DOF_GBY_KALMAN.systick += systick_reload;	
#endif

  // decide whether or not to initiate a magnetic calibration
#if defined COMPUTE_3DOF_B_BASIC || defined COMPUTE_6DOF_GB_BASIC || defined COMPUTE_9DOF_GBY_KALMAN
  // check no magnetic calibration is in progress
  if (!thisMagCal.iCalInProgress)
	{
	  // do the first 4 element calibration immediately there are a minimum of MINMEASUREMENTS4CAL
	  initiatemagcal = (!thisMagCal.iMagCalHasRun && (thisMagBuffer.iMagBufferCount >= MINMEASUREMENTS4CAL));

	  // otherwise initiate a calibration at intervals depending on the number of measurements available
	  initiatemagcal |= ((thisMagBuffer.iMagBufferCount >= MINMEASUREMENTS4CAL) && 
						 (thisMagBuffer.iMagBufferCount < MINMEASUREMENTS7CAL) &&
						 !(loopcounter % INTERVAL4CAL));
	  initiatemagcal |= ((thisMagBuffer.iMagBufferCount >= MINMEASUREMENTS7CAL) &&
						 (thisMagBuffer.iMagBufferCount < MINMEASUREMENTS10CAL) &&
						 !(loopcounter % INTERVAL7CAL));
	  initiatemagcal |= ((thisMagBuffer.iMagBufferCount >= MINMEASUREMENTS10CAL) &&
						 !(loopcounter % INTERVAL10CAL));
	} // end of test that no calibration is already in progress
  
  loopcounter++;
  
#endif
  // initiate the magnetic calibration if any of the conditions are met
  if (initiatemagcal) {
	return 1;
  }
  return 0;
}

void sfl_magnetic_calibration()
{	
  // run the magnetic calibration
#if defined COMPUTE_3DOF_B_BASIC || defined COMPUTE_6DOF_GB_BASIC || defined COMPUTE_9DOF_GBY_KALMAN
  thisMagCal.iCalInProgress = true;
  thisMagCal.iMagCalHasRun = true;
  fRunMagCalibration(&thisMagCal, &thisMagBuffer, &thisMag);
#endif
}
