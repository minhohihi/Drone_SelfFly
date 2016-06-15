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
    float                   *pRawMag = &(nMagParam.nRawMag[X_AXIS]);
    const float             nGyroOffset = DEG_TO_RAD_SCALE / GYRO_FS;

    nRawGyro[X_AXIS] = nRawGyro[X_AXIS] * nGyroOffset;
    nRawGyro[Y_AXIS] = nRawGyro[Y_AXIS] * nGyroOffset;
    nRawGyro[Z_AXIS] = nRawGyro[Z_AXIS] * nGyroOffset;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-nQuaternion[1] * nRawGyro[X_AXIS] - nQuaternion[2] * nRawGyro[Y_AXIS] - nQuaternion[3] * nRawGyro[Z_AXIS]);
    qDot2 = 0.5f * ( nQuaternion[0] * nRawGyro[X_AXIS] + nQuaternion[2] * nRawGyro[Z_AXIS] - nQuaternion[3] * nRawGyro[Y_AXIS]);
    qDot3 = 0.5f * ( nQuaternion[0] * nRawGyro[Y_AXIS] - nQuaternion[1] * nRawGyro[Z_AXIS] + nQuaternion[3] * nRawGyro[X_AXIS]);
    qDot4 = 0.5f * ( nQuaternion[0] * nRawGyro[Z_AXIS] + nQuaternion[1] * nRawGyro[Y_AXIS] - nQuaternion[2] * nRawGyro[X_AXIS]);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((nRawAccel[X_AXIS] == 0.0f) && (nRawAccel[Y_AXIS] == 0.0f) && (nRawAccel[Z_AXIS] == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = _InvSqrt(nRawAccel[X_AXIS] * nRawAccel[X_AXIS] + nRawAccel[Y_AXIS] * nRawAccel[Y_AXIS] + nRawAccel[Z_AXIS] * nRawAccel[Z_AXIS]);
        nRawAccel[X_AXIS] *= recipNorm;
        nRawAccel[Y_AXIS] *= recipNorm;
        nRawAccel[Z_AXIS] *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = _InvSqrt(pRawMag[X_AXIS] * pRawMag[X_AXIS] + pRawMag[Y_AXIS] * pRawMag[Y_AXIS] + pRawMag[Z_AXIS] * pRawMag[Z_AXIS]);
        pRawMag[X_AXIS] *= recipNorm;
        pRawMag[Y_AXIS] *= recipNorm;
        pRawMag[Z_AXIS] *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * nQuaternion[0] * pRawMag[X_AXIS];
        _2q0my = 2.0f * nQuaternion[0] * pRawMag[Y_AXIS];
        _2q0mz = 2.0f * nQuaternion[0] * pRawMag[Z_AXIS];
        _2q1mx = 2.0f * nQuaternion[1] * pRawMag[X_AXIS];
        _2q0 = 2.0f * nQuaternion[0];
        _2q1 = 2.0f * nQuaternion[1];
        _2q2 = 2.0f * nQuaternion[2];
        _2q3 = 2.0f * nQuaternion[3];
        _2q0q2 = 2.0f * nQuaternion[0] * nQuaternion[2];
        _2q2q3 = 2.0f * nQuaternion[2] * nQuaternion[3];
        q0q0 = nQuaternion[0] * nQuaternion[0];
        q0q1 = nQuaternion[0] * nQuaternion[1];
        q0q2 = nQuaternion[0] * nQuaternion[2];
        q0q3 = nQuaternion[0] * nQuaternion[3];
        q1q1 = nQuaternion[1] * nQuaternion[1];
        q1q2 = nQuaternion[1] * nQuaternion[2];
        q1q3 = nQuaternion[1] * nQuaternion[3];
        q2q2 = nQuaternion[2] * nQuaternion[2];
        q2q3 = nQuaternion[2] * nQuaternion[3];
        q3q3 = nQuaternion[3] * nQuaternion[3];

        // Reference direction of Earth's magnetic field
        hx = pRawMag[X_AXIS] * q0q0 - _2q0my * nQuaternion[3] + _2q0mz * nQuaternion[2] + pRawMag[X_AXIS] * q1q1
                    + _2q1 * pRawMag[Y_AXIS] * nQuaternion[2] + _2q1 * pRawMag[Z_AXIS] * nQuaternion[3]
                    - pRawMag[X_AXIS] * q2q2 - pRawMag[X_AXIS] * q3q3;
        hy = _2q0mx * nQuaternion[3] + pRawMag[Y_AXIS] * q0q0 - _2q0mz * nQuaternion[1] + _2q1mx * nQuaternion[2]
                    - pRawMag[Y_AXIS] * q1q1 + pRawMag[Y_AXIS] * q2q2 + _2q2 * pRawMag[Z_AXIS] * nQuaternion[3]
                    - pRawMag[Y_AXIS] * q3q3;

        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * nQuaternion[2] + _2q0my * nQuaternion[1] + pRawMag[Z_AXIS] * q0q0 + _2q1mx * nQuaternion[3]
                    - pRawMag[Z_AXIS] * q1q1 + _2q2 * pRawMag[Y_AXIS] * nQuaternion[3]
                    - pRawMag[Z_AXIS] * q2q2 + pRawMag[Z_AXIS] * q3q3;

        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - nRawAccel[X_AXIS]) + _2q1 * (2.0f * q0q1 + _2q2q3 - nRawAccel[Y_AXIS])
                    - _2bz * nQuaternion[2] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - pRawMag[X_AXIS])
                    + (-_2bx * nQuaternion[3] + _2bz * nQuaternion[1]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - pRawMag[Y_AXIS])
                    + _2bx * nQuaternion[2] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - pRawMag[Z_AXIS]);

        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - nRawAccel[X_AXIS]) + _2q0 * (2.0f * q0q1 + _2q2q3 - nRawAccel[Y_AXIS])
                    - 4.0f * nQuaternion[1] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - nRawAccel[Z_AXIS]) + _2bz * nQuaternion[3] * (_2bx * (0.5f - q2q2 - q3q3)
                    + _2bz * (q1q3 - q0q2) - pRawMag[X_AXIS]) + (_2bx * nQuaternion[2] + _2bz * nQuaternion[0]) * (_2bx * (q1q2 - q0q3)
                    + _2bz * (q0q1 + q2q3) - pRawMag[Y_AXIS]) + (_2bx * nQuaternion[3] - _4bz * nQuaternion[1]) * (_2bx * (q0q2 + q1q3)
                    + _2bz * (0.5f - q1q1 - q2q2) - pRawMag[Z_AXIS]);

        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - nRawAccel[X_AXIS]) + _2q3 * (2.0f * q0q1 + _2q2q3 - nRawAccel[Y_AXIS])
                    - 4.0f * nQuaternion[2] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - nRawAccel[Z_AXIS]) + (-_4bx * nQuaternion[2] - _2bz * nQuaternion[0]) * (_2bx * (0.5f - q2q2 - q3q3)
                    + _2bz * (q1q3 - q0q2) - pRawMag[X_AXIS]) + (_2bx * nQuaternion[1] + _2bz * nQuaternion[3]) * (_2bx * (q1q2 - q0q3)
                    + _2bz * (q0q1 + q2q3) - pRawMag[Y_AXIS]) + (_2bx * nQuaternion[0] - _4bz * nQuaternion[2]) * (_2bx * (q0q2 + q1q3)
                    + _2bz * (0.5f - q1q1 - q2q2) - pRawMag[Z_AXIS]);

        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - nRawAccel[X_AXIS]) + _2q2 * (2.0f * q0q1 + _2q2q3 - nRawAccel[Y_AXIS])
                    + (-_4bx * nQuaternion[3] + _2bz * nQuaternion[1]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - pRawMag[X_AXIS])
                    + (-_2bx * nQuaternion[0] + _2bz * nQuaternion[2]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - pRawMag[Y_AXIS])
                    + _2bx * nQuaternion[1] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - pRawMag[Z_AXIS]);

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
    nQuaternion[0] += qDot1 * nDiffTime;
    nQuaternion[1] += qDot2 * nDiffTime;
    nQuaternion[2] += qDot3 * nDiffTime;
    nQuaternion[3] += qDot4 * nDiffTime;

    // Normalise quaternion
    recipNorm = _InvSqrt(nQuaternion[0] * nQuaternion[0] + nQuaternion[1] * nQuaternion[1] + nQuaternion[2] * nQuaternion[2] + nQuaternion[3] * nQuaternion[3]);
    nQuaternion[0] *= recipNorm;
    nQuaternion[1] *= recipNorm;
    nQuaternion[2] *= recipNorm;
    nQuaternion[3] *= recipNorm;
}
#endif


void _Get_RollPitchYaw()
{
    #if 1
    nEstimatedRPY[X_AXIS] = (nEstimatedRPY[X_AXIS] * 0.8) + ((nRawGyro[X_AXIS] / 57.14286) * 0.2);    //Gyro pid input is deg/sec.
    nEstimatedRPY[Y_AXIS] = (nEstimatedRPY[Y_AXIS] * 0.8) + ((nRawGyro[Y_AXIS] / 57.14286) * 0.2);    //Gyro pid input is deg/sec.
    nEstimatedRPY[Z_AXIS] = (nEstimatedRPY[Z_AXIS] * 0.8) + ((nRawGyro[Z_AXIS] / 57.14286) * 0.2);    //Gyro pid input is deg/sec.
    #else
    // Calculate Roll & Pitch & Yaw
    _AHRSupdate();

    {
        const float nSquareQ0 = nQuaternion[0] * nQuaternion[0];
        const float nSquareQ1 = nQuaternion[1] * nQuaternion[1];
        const float nSquareGravZ = nEstGravity[Z_AXIS] * nEstGravity[Z_AXIS];

        // Estimate Gravity
        nEstGravity[X_AXIS] = 2 * ((nQuaternion[1] * nQuaternion[3]) - (nQuaternion[0] * nQuaternion[2]));
        nEstGravity[Y_AXIS] = 2 * ((nQuaternion[0] * nQuaternion[1]) + (nQuaternion[2] * nQuaternion[3]));
        nEstGravity[Z_AXIS] = (nSquareQ0) - (nSquareQ1) - (nQuaternion[2] * nQuaternion[2]) + (nQuaternion[3] * nQuaternion[3]);

        // Calculate Roll, Pitch, and Yaw
        nEstimatedRPY[0] = atan(nEstGravity[X_AXIS] / sqrt((nEstGravity[Y_AXIS] * nEstGravity[Y_AXIS]) + (nSquareGravZ))) * ((INVERSE_RPY_ROLL) ? (-1) : (1));
        nEstimatedRPY[1] = atan(nEstGravity[Y_AXIS] / sqrt((nEstGravity[X_AXIS] * nEstGravity[X_AXIS]) + (nSquareGravZ))) * ((INVERSE_RPY_PITCH) ? (-1) : (1));
        nEstimatedRPY[2] = atan2((2 * nQuaternion[1] * nQuaternion[2]) - (2 * nQuaternion[0] * nQuaternion[3]), (2 * nSquareQ0) + (2 * nSquareQ1) - 1) * ((INVERSE_RPY_YAW) ? (-1) : (1));
    }

    // Convert Radian to Degree
    nEstimatedRPY[0] *= RAD_TO_DEG_SCALE;
    nEstimatedRPY[1] *= RAD_TO_DEG_SCALE;
    nEstimatedRPY[2] *= RAD_TO_DEG_SCALE;

    if(((micros() - nInitializedTime) > RPY_OFFSET_DELAY) && (0 == bIsInitializeRPY))
    {
        bIsInitializeRPY = 1;
        nRPYOffset[0] = nEstimatedRPY[0];
        nRPYOffset[1] = nEstimatedRPY[1];
        nRPYOffset[2] = nEstimatedRPY[2];
    }
    else
    {
        nEstimatedRPY[0] -= nRPYOffset[0];
        nEstimatedRPY[1] -= nRPYOffset[1];
        nEstimatedRPY[2] -= nRPYOffset[2];
    }
    #endif
}

#endif /* AHRS_Controller_h */

