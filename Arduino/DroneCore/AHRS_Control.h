//
//  AHRS_Controller.h
//  SelfFly
//
//  Created by Maverick on 2016. 1. 21..
//

#ifndef __AHRS_CONTROL__
#define __AHRS_CONTROL__

#if 0
// Reference Site
// http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
void _AHRSupdate()
{
    float                   recipNorm;
    float                   s0, s1, s2, s3;
    float                   qDot1, qDot2, qDot3, qDot4;
    float                   hx, hy;
    float                   _2q0mx, _2q0my, _2q0mz, _2q1mx;
    float                   _2bx, _2bz, _4bx, _4bz;
    float                   _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3;
    float                   q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float                   *pRawMag = &(nMagParam._gRawMag[X_AXIS]);
    const float             nGyroOffset = DEG_TO_RAD_SCALE / GYRO_FS;

    _gRawGyro[X_AXIS] = _gRawGyro[X_AXIS] * nGyroOffset;
    _gRawGyro[Y_AXIS] = _gRawGyro[Y_AXIS] * nGyroOffset;
    _gRawGyro[Z_AXIS] = _gRawGyro[Z_AXIS] * nGyroOffset;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-_gQuat[1] * _gRawGyro[X_AXIS] - _gQuat[2] * _gRawGyro[Y_AXIS] - _gQuat[3] * _gRawGyro[Z_AXIS]);
    qDot2 = 0.5f * ( _gQuat[0] * _gRawGyro[X_AXIS] + _gQuat[2] * _gRawGyro[Z_AXIS] - _gQuat[3] * _gRawGyro[Y_AXIS]);
    qDot3 = 0.5f * ( _gQuat[0] * _gRawGyro[Y_AXIS] - _gQuat[1] * _gRawGyro[Z_AXIS] + _gQuat[3] * _gRawGyro[X_AXIS]);
    qDot4 = 0.5f * ( _gQuat[0] * _gRawGyro[Z_AXIS] + _gQuat[1] * _gRawGyro[Y_AXIS] - _gQuat[2] * _gRawGyro[X_AXIS]);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((_gRawAccel[X_AXIS] == 0.0f) && (_gRawAccel[Y_AXIS] == 0.0f) && (_gRawAccel[Z_AXIS] == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = _InvSqrt(_gRawAccel[X_AXIS] * _gRawAccel[X_AXIS] + _gRawAccel[Y_AXIS] * _gRawAccel[Y_AXIS] + _gRawAccel[Z_AXIS] * _gRawAccel[Z_AXIS]);
        _gRawAccel[X_AXIS] *= recipNorm;
        _gRawAccel[Y_AXIS] *= recipNorm;
        _gRawAccel[Z_AXIS] *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = _InvSqrt(pRawMag[X_AXIS] * pRawMag[X_AXIS] + pRawMag[Y_AXIS] * pRawMag[Y_AXIS] + pRawMag[Z_AXIS] * pRawMag[Z_AXIS]);
        pRawMag[X_AXIS] *= recipNorm;
        pRawMag[Y_AXIS] *= recipNorm;
        pRawMag[Z_AXIS] *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * _gQuat[0] * pRawMag[X_AXIS];
        _2q0my = 2.0f * _gQuat[0] * pRawMag[Y_AXIS];
        _2q0mz = 2.0f * _gQuat[0] * pRawMag[Z_AXIS];
        _2q1mx = 2.0f * _gQuat[1] * pRawMag[X_AXIS];
        _2q0 = 2.0f * _gQuat[0];
        _2q1 = 2.0f * _gQuat[1];
        _2q2 = 2.0f * _gQuat[2];
        _2q3 = 2.0f * _gQuat[3];
        _2q0q2 = 2.0f * _gQuat[0] * _gQuat[2];
        _2q2q3 = 2.0f * _gQuat[2] * _gQuat[3];
        q0q0 = _gQuat[0] * _gQuat[0];
        q0q1 = _gQuat[0] * _gQuat[1];
        q0q2 = _gQuat[0] * _gQuat[2];
        q0q3 = _gQuat[0] * _gQuat[3];
        q1q1 = _gQuat[1] * _gQuat[1];
        q1q2 = _gQuat[1] * _gQuat[2];
        q1q3 = _gQuat[1] * _gQuat[3];
        q2q2 = _gQuat[2] * _gQuat[2];
        q2q3 = _gQuat[2] * _gQuat[3];
        q3q3 = _gQuat[3] * _gQuat[3];

        // Reference direction of Earth's magnetic field
        hx = pRawMag[X_AXIS] * q0q0 - _2q0my * _gQuat[3] + _2q0mz * _gQuat[2] + pRawMag[X_AXIS] * q1q1
                    + _2q1 * pRawMag[Y_AXIS] * _gQuat[2] + _2q1 * pRawMag[Z_AXIS] * _gQuat[3]
                    - pRawMag[X_AXIS] * q2q2 - pRawMag[X_AXIS] * q3q3;
        hy = _2q0mx * _gQuat[3] + pRawMag[Y_AXIS] * q0q0 - _2q0mz * _gQuat[1] + _2q1mx * _gQuat[2]
                    - pRawMag[Y_AXIS] * q1q1 + pRawMag[Y_AXIS] * q2q2 + _2q2 * pRawMag[Z_AXIS] * _gQuat[3]
                    - pRawMag[Y_AXIS] * q3q3;

        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * _gQuat[2] + _2q0my * _gQuat[1] + pRawMag[Z_AXIS] * q0q0 + _2q1mx * _gQuat[3]
                    - pRawMag[Z_AXIS] * q1q1 + _2q2 * pRawMag[Y_AXIS] * _gQuat[3]
                    - pRawMag[Z_AXIS] * q2q2 + pRawMag[Z_AXIS] * q3q3;

        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - _gRawAccel[X_AXIS]) + _2q1 * (2.0f * q0q1 + _2q2q3 - _gRawAccel[Y_AXIS])
                    - _2bz * _gQuat[2] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - pRawMag[X_AXIS])
                    + (-_2bx * _gQuat[3] + _2bz * _gQuat[1]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - pRawMag[Y_AXIS])
                    + _2bx * _gQuat[2] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - pRawMag[Z_AXIS]);

        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - _gRawAccel[X_AXIS]) + _2q0 * (2.0f * q0q1 + _2q2q3 - _gRawAccel[Y_AXIS])
                    - 4.0f * _gQuat[1] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - _gRawAccel[Z_AXIS]) + _2bz * _gQuat[3] * (_2bx * (0.5f - q2q2 - q3q3)
                    + _2bz * (q1q3 - q0q2) - pRawMag[X_AXIS]) + (_2bx * _gQuat[2] + _2bz * _gQuat[0]) * (_2bx * (q1q2 - q0q3)
                    + _2bz * (q0q1 + q2q3) - pRawMag[Y_AXIS]) + (_2bx * _gQuat[3] - _4bz * _gQuat[1]) * (_2bx * (q0q2 + q1q3)
                    + _2bz * (0.5f - q1q1 - q2q2) - pRawMag[Z_AXIS]);

        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - _gRawAccel[X_AXIS]) + _2q3 * (2.0f * q0q1 + _2q2q3 - _gRawAccel[Y_AXIS])
                    - 4.0f * _gQuat[2] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - _gRawAccel[Z_AXIS]) + (-_4bx * _gQuat[2] - _2bz * _gQuat[0]) * (_2bx * (0.5f - q2q2 - q3q3)
                    + _2bz * (q1q3 - q0q2) - pRawMag[X_AXIS]) + (_2bx * _gQuat[1] + _2bz * _gQuat[3]) * (_2bx * (q1q2 - q0q3)
                    + _2bz * (q0q1 + q2q3) - pRawMag[Y_AXIS]) + (_2bx * _gQuat[0] - _4bz * _gQuat[2]) * (_2bx * (q0q2 + q1q3)
                    + _2bz * (0.5f - q1q1 - q2q2) - pRawMag[Z_AXIS]);

        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - _gRawAccel[X_AXIS]) + _2q2 * (2.0f * q0q1 + _2q2q3 - _gRawAccel[Y_AXIS])
                    + (-_4bx * _gQuat[3] + _2bz * _gQuat[1]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - pRawMag[X_AXIS])
                    + (-_2bx * _gQuat[0] + _2bz * _gQuat[2]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - pRawMag[Y_AXIS])
                    + _2bx * _gQuat[1] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - pRawMag[Z_AXIS]);

        recipNorm = _InvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= nBeta * s0;
        qDot2 -= nBeta * s1;
        qDot3 -= nBeta * s2;
        qDot4 -= nBeta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    _gQuat[0] += qDot1 * _gDiffTime;
    _gQuat[1] += qDot2 * _gDiffTime;
    _gQuat[2] += qDot3 * _gDiffTime;
    _gQuat[3] += qDot4 * _gDiffTime;

    // Normalise quaternion
    recipNorm = _InvSqrt(_gQuat[0] * _gQuat[0] + _gQuat[1] * _gQuat[1] + _gQuat[2] * _gQuat[2] + _gQuat[3] * _gQuat[3]);
    _gQuat[0] *= recipNorm;
    _gQuat[1] *= recipNorm;
    _gQuat[2] *= recipNorm;
    _gQuat[3] *= recipNorm;
}
#endif


void _Get_RollPitchYaw()
{
    #if 1
        #if USE_MPU6050_DMP
            _AccelGyro_GetRPY();

            _gEstimatedRPY[X_AXIS] = ypr[2] * RAD_TO_DEG_SCALE;         // Roll
            _gEstimatedRPY[Y_AXIS] = ypr[1] * RAD_TO_DEG_SCALE;         // Pitch
            _gEstimatedRPY[Z_AXIS] = ypr[0] * RAD_TO_DEG_SCALE;         // Yaw
        #else
            _gEstimatedRPY[X_AXIS] = (_gEstimatedRPY[X_AXIS] * 0.8) + ((_gRawGyro[X_AXIS] / 57.14286) * 0.2);    //Gyro pid input is deg/sec.
            _gEstimatedRPY[Y_AXIS] = (_gEstimatedRPY[Y_AXIS] * 0.8) + ((_gRawGyro[Y_AXIS] / 57.14286) * 0.2);    //Gyro pid input is deg/sec.
            _gEstimatedRPY[Z_AXIS] = (_gEstimatedRPY[Z_AXIS] * 0.8) + ((_gRawGyro[Z_AXIS] / 57.14286) * 0.2);    //Gyro pid input is deg/sec.
        #endif
    #else
        // Calculate Roll & Pitch & Yaw
        _AHRSupdate();

        {
            const float nSquareQ0 = _gQuat[0] * _gQuat[0];
            const float nSquareQ1 = _gQuat[1] * _gQuat[1];
            const float nSquareGravZ = _gEstGravity[Z_AXIS] * _gEstGravity[Z_AXIS];

            // Estimate Gravity
            _gEstGravity[X_AXIS] = 2 * ((_gQuat[1] * _gQuat[3]) - (_gQuat[0] * _gQuat[2]));
            _gEstGravity[Y_AXIS] = 2 * ((_gQuat[0] * _gQuat[1]) + (_gQuat[2] * _gQuat[3]));
            _gEstGravity[Z_AXIS] = (nSquareQ0) - (nSquareQ1) - (_gQuat[2] * _gQuat[2]) + (_gQuat[3] * _gQuat[3]);

            // Calculate Roll, Pitch, and Yaw
            _gEstimatedRPY[0] = atan(_gEstGravity[X_AXIS] / sqrt((_gEstGravity[Y_AXIS] * _gEstGravity[Y_AXIS]) + (nSquareGravZ))) * ((INVERSE_RPY_ROLL) ? (-1) : (1));
            _gEstimatedRPY[1] = atan(_gEstGravity[Y_AXIS] / sqrt((_gEstGravity[X_AXIS] * _gEstGravity[X_AXIS]) + (nSquareGravZ))) * ((INVERSE_RPY_PITCH) ? (-1) : (1));
            _gEstimatedRPY[2] = atan2((2 * _gQuat[1] * _gQuat[2]) - (2 * _gQuat[0] * _gQuat[3]), (2 * nSquareQ0) + (2 * nSquareQ1) - 1) * ((INVERSE_RPY_YAW) ? (-1) : (1));
        }

        // Convert Radian to Degree
        _gEstimatedRPY[0] *= RAD_TO_DEG_SCALE;
        _gEstimatedRPY[1] *= RAD_TO_DEG_SCALE;
        _gEstimatedRPY[2] *= RAD_TO_DEG_SCALE;

        if(((micros() - nInitializedTime) > RPY_OFFSET_DELAY) && (0 == _gbIsInitializeRPY))
        {
            _gbIsInitializeRPY = 1;
            _gRPYOffset[0] = _gEstimatedRPY[0];
            _gRPYOffset[1] = _gEstimatedRPY[1];
            _gRPYOffset[2] = _gEstimatedRPY[2];
        }
        else
        {
            _gEstimatedRPY[0] -= _gRPYOffset[0];
            _gEstimatedRPY[1] -= _gRPYOffset[1];
            _gEstimatedRPY[2] -= _gRPYOffset[2];
        }
    #endif
}

#endif /* AHRS_Controller_h */

