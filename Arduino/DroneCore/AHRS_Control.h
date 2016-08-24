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
    int             i = 0;
    
    for(i=0 ; i<3 ; i++)
    {
        // Reverse MPU Data If Needed
        if(1 == _gMPUAxisRvrsFlag[_gGyroAccelAxis[i]])
        {
            _gRawGyro[_gGyroAccelAxis[i]] *= -1.0;
            _gRawAccel[_gGyroAccelAxis[i]] *= -1.0;
        }
    }
    
    _gEstRoll  = (_gEstRoll * nOffset0) + ((_gRawGyro[_gGyroAccelAxis[0]] / GYRO_FS) * nOffset1);   //Gyro pid input is deg/sec.
    _gEstPitch = (_gEstPitch * nOffset0) + ((_gRawGyro[_gGyroAccelAxis[1]] / GYRO_FS) * nOffset1);  //Gyro pid input is deg/sec.
    _gEstYaw   = (_gEstYaw * nOffset0) + ((_gRawGyro[_gGyroAccelAxis[2]] / GYRO_FS) * nOffset1);    //Gyro pid input is deg/sec.
    
    // Gyro angle calculations
    //0.0000611 = 1 / (250Hz / GYRO_FS=65.5)
    _gAngleRoll += _gRawGyro[_gGyroAccelAxis[0]] * 0.0000611;                                   //Calculate the traveled roll angle and add this to the angle_roll variable.
    _gAnglePitch += _gRawGyro[_gGyroAccelAxis[1]] * 0.0000611;                                  //Calculate the traveled pitch angle and add this to the angle_pitch variable.
    
    // 0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
    _gAnglePitch += _gAngleRoll * sin(_gRawGyro[_gGyroAccelAxis[2]] * 0.000001066);             //If the IMU has yawed transfer the roll angle to the pitch angel.
    _gAngleRoll -= _gAnglePitch * sin(_gRawGyro[_gGyroAccelAxis[2]] * 0.000001066);             //If the IMU has yawed transfer the pitch angle to the roll angel.
    
    // Accelerometer angle calculations
    // Calculate the total accelerometer vector.
    _gAccTotalVector = sqrt((_gRawAccel[X_AXIS] * _gRawAccel[X_AXIS]) +
                            (_gRawAccel[Y_AXIS] * _gRawAccel[Y_AXIS]) +
                            (_gRawAccel[Z_AXIS] * _gRawAccel[Z_AXIS]));

    //if(abs(_gRawAccel[X_AXIS]) < _gAccTotalVector)
    _gAngleRollAcc = asin((float)_gRawAccel[_gGyroAccelAxis[0]] / _gAccTotalVector) * (-RAD_TO_DEG_SCALE);
    
    //if(abs(_gRawAccel[Y_AXIS]) < _gAccTotalVector)
    _gAnglePitchAcc = asin((float)_gRawAccel[_gGyroAccelAxis[1]] / _gAccTotalVector) * RAD_TO_DEG_SCALE;

    //Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
    _gAnglePitchAcc -= 0.0;
    _gAngleRollAcc -= 0.0;
    
    if(true == _gbAngleSet)
    {                                                 //If the IMU is already started
        _gAnglePitch = _gAnglePitch * 0.9996 + _gAnglePitchAcc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
        _gAngleRoll = _gAngleRoll * 0.9996 + _gAngleRollAcc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
    }
    else
    {                                                                //At first start
        _gAnglePitch = _gAnglePitchAcc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
        _gAngleRoll = _gAngleRollAcc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
        _gbAngleSet = true;                                            //Set the IMU started flag
    }
  
    _gAnglePitchOut = _gAnglePitchOut * 0.9 + _gAnglePitch * 0.1;                   //Take 90% of the output pitch value and add 10% of the raw pitch value
    _gAngleRollOut = _gAngleRollOut * 0.9 + _gAngleRoll * 0.1;                      //Take 90% of the output roll value and add 10% of the raw roll value
    
    _gPitchLevelAdjust = _gAnglePitch * 15.0;
    _gRollLevelAdjust = _gAngleRoll * 15.0;
    
    if(0 == USE_AUTO_LEVEL)
    {
        _gPitchLevelAdjust = 0.0;
        _gRollLevelAdjust = 0.0;
    }
}

#endif /* AHRS_Controller_h */



