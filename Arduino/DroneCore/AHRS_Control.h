//
//  AHRS_Controller.h
//  SelfFly
//
//  Created by Maverick on 2016. 1. 21..
//

#ifndef __AHRS_CONTROL__
#define __AHRS_CONTROL__

void _Get_RollPitchYaw()
{
    const float     nOffset0 = 0.9f;
    const float     nOffset1 = 1.0f  - nOffset0;
    int             i = 0;
    const int       nRollAxisIdx  = _gGyroAccelAxis[0];
    const int       nPitchAxisIdx = _gGyroAccelAxis[1];
    const int       nYawAxisIdx   = _gGyroAccelAxis[2];

    for(i=0 ; i<3 ; i++)
    {
        // Reverse MPU Data If Needed
        if(1 == _gMPUAxisRvrsFlag[_gGyroAccelAxis[i]])
        {
            _gRawGyro[_gGyroAccelAxis[i]] *= -1.0;
            _gRawAccel[_gGyroAccelAxis[i]] *= -1.0;
        }
    }
    
    // Gyro angle calculations
    //0.000076335 = 1 / (200Hz / GYRO_FS=65.5)
    _gAnglePitch += _gRawGyro[nPitchAxisIdx] * ACCELGYRO_FS;
    _gAngleRoll  += _gRawGyro[nRollAxisIdx] * ACCELGYRO_FS;
    _gAngleYaw   = (_gAngleYaw * 0.7) + ((_gRawGyro[_gGyroAccelAxis[2]] / GYRO_FS) * 0.3);

    // 0.00000133229 = ACCELGYRO_FS * (3.142(PI) / 180degr) The Arduino sin function is in radians
    _gAnglePitch +=  _gAngleRoll * sin(_gRawGyro[nYawAxisIdx] * APPROX_SIN_SCALE);
    _gAngleRoll  -= _gAnglePitch * sin(_gRawGyro[nYawAxisIdx] * APPROX_SIN_SCALE);
    
    // Accelerometer angle calculations
    // Calculate the total accelerometer vector.
    _gAccTotalVector = sqrt((_gRawAccel[X_AXIS] * _gRawAccel[X_AXIS]) +
                            (_gRawAccel[Y_AXIS] * _gRawAccel[Y_AXIS]) +
                            (_gRawAccel[Z_AXIS] * _gRawAccel[Z_AXIS]));

    _gAnglePitchAcc = asin(_gRawAccel[nPitchAxisIdx] / _gAccTotalVector) * RAD_TO_DEG_SCALE;
    _gAngleRollAcc  = asin(_gRawAccel[nRollAxisIdx] / _gAccTotalVector) * (-RAD_TO_DEG_SCALE);

    //Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
    _gAnglePitchAcc -= 0.0;
    _gAngleRollAcc -= 0.0;
    
    if(true == _gbAngleSet)
    {
        _gAnglePitch = _gAnglePitch * 0.9996 + _gAnglePitchAcc * 0.0004;
        _gAngleRoll = _gAngleRoll * 0.9996 + _gAngleRollAcc * 0.0004;
    }
    else
    {
        _gAnglePitch = _gAnglePitchAcc;
        _gAngleRoll = _gAngleRollAcc;
        _gbAngleSet = true;
    }
  
    _gAnglePitchOut = _gAnglePitchOut * nOffset0 + _gAnglePitch * nOffset1;
    _gAngleRollOut  = _gAngleRollOut * nOffset0 + _gAngleRoll * nOffset1;
    
    _gEstimatedRPY[0] = _gAngleRollOut;
    _gEstimatedRPY[1] = _gAnglePitchOut;
    _gEstimatedRPY[2] = _gAngleYaw;
    
    _gPitchLevelAdjust = _gAnglePitch * 15.0;
    _gRollLevelAdjust = _gAngleRoll * 15.0;
    
    if(0 == USE_AUTO_LEVEL)
    {
        _gPitchLevelAdjust = 0.0;
        _gRollLevelAdjust = 0.0;
    }
}

#endif /* AHRS_Controller_h */

















