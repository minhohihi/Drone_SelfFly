//
//  AHRS_Controller.h
//  SelfFly
//
//  Created by Maverick on 2016. 1. 21..
//

#ifndef __AHRS_CONTROL__
#define __AHRS_CONTROL__

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
    const float             nDiffTime = pSelfFlyHndl->nDiffTime;
    float                   *pQ = &(pSelfFlyHndl->nQuaternion[0]);
    float                   *pRawGyro = &(pSelfFlyHndl->nAccelGyroParam.nRawGyro[X_AXIS]);
    float                   *pRawAccel = &(pSelfFlyHndl->nAccelGyroParam.nRawAccel[X_AXIS]);
    float                   *pRawMag = &(pSelfFlyHndl->nMagParam.nRawMag[X_AXIS]);
    const float             nGyroOffset = DEG_TO_RAD_SCALE / GYRO_FS;

    pRawGyro[X_AXIS] = pRawGyro[X_AXIS] * nGyroOffset;
    pRawGyro[Y_AXIS] = pRawGyro[Y_AXIS] * nGyroOffset;
    pRawGyro[Z_AXIS] = pRawGyro[Z_AXIS] * nGyroOffset;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-pQ[1] * pRawGyro[X_AXIS] - pQ[2] * pRawGyro[Y_AXIS] - pQ[3] * pRawGyro[Z_AXIS]);
    qDot2 = 0.5f * ( pQ[0] * pRawGyro[X_AXIS] + pQ[2] * pRawGyro[Z_AXIS] - pQ[3] * pRawGyro[Y_AXIS]);
    qDot3 = 0.5f * ( pQ[0] * pRawGyro[Y_AXIS] - pQ[1] * pRawGyro[Z_AXIS] + pQ[3] * pRawGyro[X_AXIS]);
    qDot4 = 0.5f * ( pQ[0] * pRawGyro[Z_AXIS] + pQ[1] * pRawGyro[Y_AXIS] - pQ[2] * pRawGyro[X_AXIS]);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((pRawAccel[X_AXIS] == 0.0f) && (pRawAccel[Y_AXIS] == 0.0f) && (pRawAccel[Z_AXIS] == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = _InvSqrt(pRawAccel[X_AXIS] * pRawAccel[X_AXIS] + pRawAccel[Y_AXIS] * pRawAccel[Y_AXIS] + pRawAccel[Z_AXIS] * pRawAccel[Z_AXIS]);
        pRawAccel[X_AXIS] *= recipNorm;
        pRawAccel[Y_AXIS] *= recipNorm;
        pRawAccel[Z_AXIS] *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = _InvSqrt(pRawMag[X_AXIS] * pRawMag[X_AXIS] + pRawMag[Y_AXIS] * pRawMag[Y_AXIS] + pRawMag[Z_AXIS] * pRawMag[Z_AXIS]);
        pRawMag[X_AXIS] *= recipNorm;
        pRawMag[Y_AXIS] *= recipNorm;
        pRawMag[Z_AXIS] *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * pQ[0] * pRawMag[X_AXIS];
        _2q0my = 2.0f * pQ[0] * pRawMag[Y_AXIS];
        _2q0mz = 2.0f * pQ[0] * pRawMag[Z_AXIS];
        _2q1mx = 2.0f * pQ[1] * pRawMag[X_AXIS];
        _2q0 = 2.0f * pQ[0];
        _2q1 = 2.0f * pQ[1];
        _2q2 = 2.0f * pQ[2];
        _2q3 = 2.0f * pQ[3];
        _2q0q2 = 2.0f * pQ[0] * pQ[2];
        _2q2q3 = 2.0f * pQ[2] * pQ[3];
        q0q0 = pQ[0] * pQ[0];
        q0q1 = pQ[0] * pQ[1];
        q0q2 = pQ[0] * pQ[2];
        q0q3 = pQ[0] * pQ[3];
        q1q1 = pQ[1] * pQ[1];
        q1q2 = pQ[1] * pQ[2];
        q1q3 = pQ[1] * pQ[3];
        q2q2 = pQ[2] * pQ[2];
        q2q3 = pQ[2] * pQ[3];
        q3q3 = pQ[3] * pQ[3];

        // Reference direction of Earth's magnetic field
        hx = pRawMag[X_AXIS] * q0q0 - _2q0my * pQ[3] + _2q0mz * pQ[2] + pRawMag[X_AXIS] * q1q1
                    + _2q1 * pRawMag[Y_AXIS] * pQ[2] + _2q1 * pRawMag[Z_AXIS] * pQ[3]
                    - pRawMag[X_AXIS] * q2q2 - pRawMag[X_AXIS] * q3q3;
        hy = _2q0mx * pQ[3] + pRawMag[Y_AXIS] * q0q0 - _2q0mz * pQ[1] + _2q1mx * pQ[2]
                    - pRawMag[Y_AXIS] * q1q1 + pRawMag[Y_AXIS] * q2q2 + _2q2 * pRawMag[Z_AXIS] * pQ[3]
                    - pRawMag[Y_AXIS] * q3q3;

        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * pQ[2] + _2q0my * pQ[1] + pRawMag[Z_AXIS] * q0q0 + _2q1mx * pQ[3]
                    - pRawMag[Z_AXIS] * q1q1 + _2q2 * pRawMag[Y_AXIS] * pQ[3]
                    - pRawMag[Z_AXIS] * q2q2 + pRawMag[Z_AXIS] * q3q3;

        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - pRawAccel[X_AXIS]) + _2q1 * (2.0f * q0q1 + _2q2q3 - pRawAccel[Y_AXIS])
                    - _2bz * pQ[2] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - pRawMag[X_AXIS])
                    + (-_2bx * pQ[3] + _2bz * pQ[1]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - pRawMag[Y_AXIS])
                    + _2bx * pQ[2] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - pRawMag[Z_AXIS]);

        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - pRawAccel[X_AXIS]) + _2q0 * (2.0f * q0q1 + _2q2q3 - pRawAccel[Y_AXIS])
                    - 4.0f * pQ[1] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - pRawAccel[Z_AXIS]) + _2bz * pQ[3] * (_2bx * (0.5f - q2q2 - q3q3)
                    + _2bz * (q1q3 - q0q2) - pRawMag[X_AXIS]) + (_2bx * pQ[2] + _2bz * pQ[0]) * (_2bx * (q1q2 - q0q3)
                    + _2bz * (q0q1 + q2q3) - pRawMag[Y_AXIS]) + (_2bx * pQ[3] - _4bz * pQ[1]) * (_2bx * (q0q2 + q1q3)
                    + _2bz * (0.5f - q1q1 - q2q2) - pRawMag[Z_AXIS]);

        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - pRawAccel[X_AXIS]) + _2q3 * (2.0f * q0q1 + _2q2q3 - pRawAccel[Y_AXIS])
                    - 4.0f * pQ[2] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - pRawAccel[Z_AXIS]) + (-_4bx * pQ[2] - _2bz * pQ[0]) * (_2bx * (0.5f - q2q2 - q3q3)
                    + _2bz * (q1q3 - q0q2) - pRawMag[X_AXIS]) + (_2bx * pQ[1] + _2bz * pQ[3]) * (_2bx * (q1q2 - q0q3)
                    + _2bz * (q0q1 + q2q3) - pRawMag[Y_AXIS]) + (_2bx * pQ[0] - _4bz * pQ[2]) * (_2bx * (q0q2 + q1q3)
                    + _2bz * (0.5f - q1q1 - q2q2) - pRawMag[Z_AXIS]);

        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - pRawAccel[X_AXIS]) + _2q2 * (2.0f * q0q1 + _2q2q3 - pRawAccel[Y_AXIS])
                    + (-_4bx * pQ[3] + _2bz * pQ[1]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - pRawMag[X_AXIS])
                    + (-_2bx * pQ[0] + _2bz * pQ[2]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - pRawMag[Y_AXIS])
                    + _2bx * pQ[1] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - pRawMag[Z_AXIS]);

        recipNorm = _InvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= pSelfFlyHndl->nBeta * s0;
        qDot2 -= pSelfFlyHndl->nBeta * s1;
        qDot3 -= pSelfFlyHndl->nBeta * s2;
        qDot4 -= pSelfFlyHndl->nBeta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    pQ[0] += qDot1 * nDiffTime;
    pQ[1] += qDot2 * nDiffTime;
    pQ[2] += qDot3 * nDiffTime;
    pQ[3] += qDot4 * nDiffTime;

    // Normalise quaternion
    recipNorm = _InvSqrt(pQ[0] * pQ[0] + pQ[1] * pQ[1] + pQ[2] * pQ[2] + pQ[3] * pQ[3]);
    pQ[0] *= recipNorm;
    pQ[1] *= recipNorm;
    pQ[2] *= recipNorm;
    pQ[3] *= recipNorm;
}


void _Get_RollPitchYaw()
{
    float           *pEstGravity = &(pSelfFlyHndl->nEstGravity[X_AXIS]);
    float           *pQ = &(pSelfFlyHndl->nQuaternion[0]);
    float           *pFineRPY = &(pSelfFlyHndl->nFineRPY[0]);

    // Calculate Roll & Pitch & Yaw
    _AHRSupdate();

    {
        const float nSquareQ0 = pQ[0] * pQ[0];
        const float nSquareQ1 = pQ[1] * pQ[1];
        const float nSquareGravZ = pEstGravity[Z_AXIS] * pEstGravity[Z_AXIS];

        // Estimate Gravity
        pEstGravity[X_AXIS] = 2 * ((pQ[1] * pQ[3]) - (pQ[0] * pQ[2]));
        pEstGravity[Y_AXIS] = 2 * ((pQ[0] * pQ[1]) + (pQ[2] * pQ[3]));
        pEstGravity[Z_AXIS] = (nSquareQ0) - (nSquareQ1) - (pQ[2] * pQ[2]) + (pQ[3] * pQ[3]);

        // Calculate Roll, Pitch, and Yaw
        pFineRPY[0] = atan(pEstGravity[X_AXIS] / sqrt((pEstGravity[Y_AXIS] * pEstGravity[Y_AXIS]) + (nSquareGravZ))) * ((INVERSE_RPY_ROLL) ? (-1) : (1));
        pFineRPY[1] = atan(pEstGravity[Y_AXIS] / sqrt((pEstGravity[X_AXIS] * pEstGravity[X_AXIS]) + (nSquareGravZ))) * ((INVERSE_RPY_PITCH) ? (-1) : (1));
        pFineRPY[2] = atan2((2 * pQ[1] * pQ[2]) - (2 * pQ[0] * pQ[3]), (2 * nSquareQ0) + (2 * nSquareQ1) - 1) * ((INVERSE_RPY_YAW) ? (-1) : (1));
    }

    // Convert Radian to Degree
    pFineRPY[0] *= RAD_TO_DEG_SCALE;
    pFineRPY[1] *= RAD_TO_DEG_SCALE;
    pFineRPY[2] *= RAD_TO_DEG_SCALE;

    if(((micros() - pSelfFlyHndl->nInitializedTime) > RPY_OFFSET_DELAY) && (0 == pSelfFlyHndl->bIsInitializeRPY))
    {
        pSelfFlyHndl->bIsInitializeRPY = 1;
        pSelfFlyHndl->nRPYOffset[0] = pFineRPY[0];
        pSelfFlyHndl->nRPYOffset[1] = pFineRPY[1];
        pSelfFlyHndl->nRPYOffset[2] = pFineRPY[2];
    }
    else
    {
        pFineRPY[0] -= pSelfFlyHndl->nRPYOffset[0];
        pFineRPY[1] -= pSelfFlyHndl->nRPYOffset[1];
        pFineRPY[2] -= pSelfFlyHndl->nRPYOffset[2];
    }
}

#endif /* AHRS_Controller_h */

