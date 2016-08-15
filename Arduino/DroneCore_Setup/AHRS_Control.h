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
    const float     nOffset0 = 0.7f;
    const float     nOffset1 = 1.0f  - nOffset0;
    
    _gEstimatedRPY[X_AXIS] = (_gEstimatedRPY[X_AXIS] * nOffset0) + ((_gRawGyro[X_AXIS] / GYRO_FS) * nOffset1);    //Gyro pid input is deg/sec.
    _gEstimatedRPY[Y_AXIS] = (_gEstimatedRPY[Y_AXIS] * nOffset0) + ((_gRawGyro[Y_AXIS] / GYRO_FS) * nOffset1);    //Gyro pid input is deg/sec.
    _gEstimatedRPY[Z_AXIS] = (_gEstimatedRPY[Z_AXIS] * nOffset0) + ((_gRawGyro[Z_AXIS] / GYRO_FS) * nOffset1);    //Gyro pid input is deg/sec.
    
    // Gyro angle calculations
    //0.0000611 = 1 / (250Hz / GYRO_FS=65.5)
    _gAnglePitch += _gRawGyro[Y_AXIS] * 0.0000611;                                  //Calculate the traveled pitch angle and add this to the angle_pitch variable.
    _gAngleRoll += _gRawGyro[X_AXIS] * 0.0000611;                                   //Calculate the traveled roll angle and add this to the angle_roll variable.
    _gAngleYaw += _gRawGyro[Z_AXIS] * 0.0000611;                                   //Calculate the traveled roll angle and add this to the angle_roll variable.
    
    // 0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
    _gAnglePitch -= _gAngleRoll * sin(_gRawGyro[Z_AXIS] * 0.000001066);             //If the IMU has yawed transfer the roll angle to the pitch angel.
    _gAngleRoll += _gAnglePitch * sin(_gRawGyro[Z_AXIS] * 0.000001066);             //If the IMU has yawed transfer the pitch angle to the roll angel.
    
    // Accelerometer angle calculations
    // Calculate the total accelerometer vector.
    _gAccTotalVector = sqrt((_gRawAccel[X_AXIS] * _gRawAccel[X_AXIS]) + (_gRawAccel[Y_AXIS] * _gRawAccel[Y_AXIS]) + (_gRawAccel[Z_AXIS] * _gRawAccel[Z_AXIS]));

    if(abs(_gRawAccel[X_AXIS]) < _gAccTotalVector)
        _gAngleRollAcc = asin((float)_gRawAccel[X_AXIS] / _gAccTotalVector) * (-RAD_TO_DEG_SCALE);
    
    if(abs(_gRawAccel[Y_AXIS]) < _gAccTotalVector)
        _gAnglePitchAcc = asin((float)_gRawAccel[Y_AXIS] / _gAccTotalVector) * RAD_TO_DEG_SCALE;

    //Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
    _gAnglePitchAcc -= 0.0;
    _gAngleRollAcc -= 0.0;
    
    _gAnglePitch = _gAnglePitch * 0.9996 + _gAnglePitchAcc * 0.0004;                //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
    _gAngleRoll = _gAngleRoll * 0.9996 + _gAngleRollAcc * 0.0004;                   //Correct the drift of the gyro roll angle with the accelerometer roll angle.
    
    _gPitchLevelAdjust = _gAnglePitch * 15.0;
    _gRollLevelAdjust = _gAngleRoll * 15.0;
    
    if(1 == USE_AUTO_LEVEL)
    {
        _gPitchLevelAdjust = 0.0;
        _gRollLevelAdjust = 0.0;
    }
}

#endif /* AHRS_Controller_h */


