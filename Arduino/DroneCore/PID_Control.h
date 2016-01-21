//
//  PID_Controller.h
//  SelfFly
//
//  Created by Maverick on 2016. 1. 21..
//

#ifndef __PID_CONTROL__
#define __PID_CONTROL__

inline void _CalculatePID()
{
    static long             nPrevRCVal[5] = {0, };              // Filter variables
    static float            nPrevFineRPY[3];
    AxisErrRate_T           *pPitch = &(pSelfFlyHndl->nPitch);
    AxisErrRate_T           *pRoll = &(pSelfFlyHndl->nRoll);
    AxisErrRate_T           *pYaw = &(pSelfFlyHndl->nYaw);
    long                    *pUsingRCVal = &(pSelfFlyHndl->nUsingRCVal[0]);
    float                   *pFineGyro = &(pSelfFlyHndl->nFineGyro[0]);
    float                   *pFineRPY = &(pSelfFlyHndl->nFineRPY[0]);
    const float             nDiffTime = pSelfFlyHndl->nDiffTime;

    pSelfFlyHndl->nLock = true;
    memcpy(pUsingRCVal, &(pSelfFlyHndl->nCapturedRCVal[0]), MAX_CH_RC * sizeof(long));
    pSelfFlyHndl->nLock = false;
    
    pUsingRCVal[CH_TYPE_ROLL] = floor(pUsingRCVal[CH_TYPE_ROLL] / ROUNDING_BASE) * ROUNDING_BASE;
    pUsingRCVal[CH_TYPE_PITCH] = floor(pUsingRCVal[CH_TYPE_PITCH] / ROUNDING_BASE) * ROUNDING_BASE;
    pUsingRCVal[CH_TYPE_YAW] = floor(pUsingRCVal[CH_TYPE_YAW] / ROUNDING_BASE) * ROUNDING_BASE;

    pUsingRCVal[CH_TYPE_ROLL] = map(pUsingRCVal[CH_TYPE_ROLL], RC_CH0_LOW, RC_CH0_HIGH, ROLL_ANG_MIN, ROLL_ANG_MAX) - 1;
    pUsingRCVal[CH_TYPE_PITCH] = map(pUsingRCVal[CH_TYPE_PITCH], RC_CH1_LOW, RC_CH1_HIGH, PITCH_ANG_MIN, PITCH_ANG_MAX) - 1;
    pUsingRCVal[CH_TYPE_YAW] = map(pUsingRCVal[CH_TYPE_YAW], RC_CH3_LOW, RC_CH3_HIGH, YAW_RATE_MIN, YAW_RATE_MAX);

    //if((pUsingRCVal[CH_TYPE_ROLL] < ROLL_ANG_MIN) || (pUsingRCVal[CH_TYPE_ROLL] > ROLL_ANG_MAX))
    //    pUsingRCVal[CH_TYPE_ROLL] = nPrevRCVal[CH_TYPE_ROLL];
    
    //if((pUsingRCVal[CH_TYPE_PITCH] < PITCH_ANG_MIN) || (pUsingRCVal[CH_TYPE_PITCH] > PITCH_ANG_MAX))
    //    pUsingRCVal[CH_TYPE_PITCH] = nPrevRCVal[CH_TYPE_PITCH];
    
    //if((pUsingRCVal[CH_TYPE_YAW] < YAW_RATE_MIN) || (pUsingRCVal[CH_TYPE_YAW] > YAW_RATE_MAX))
    //    pUsingRCVal[CH_TYPE_YAW] = nPrevRCVal[CH_TYPE_YAW];
    
    nPrevRCVal[CH_TYPE_ROLL] = pUsingRCVal[CH_TYPE_ROLL];
    nPrevRCVal[CH_TYPE_PITCH] = pUsingRCVal[CH_TYPE_PITCH];
    nPrevRCVal[CH_TYPE_YAW] = pUsingRCVal[CH_TYPE_YAW];

    if(abs(pFineRPY[0] - nPrevFineRPY[0]) > 30)
        pFineRPY[0] = nPrevFineRPY[0];

    if(abs(pFineRPY[1] - nPrevFineRPY[1]) > 30)
        pFineRPY[1] = nPrevFineRPY[1];

    //ROLL control
    pRoll->nAngleErr = pUsingRCVal[CH_TYPE_ROLL] - pFineRPY[0];
    pRoll->nCurrErrRate = pRoll->nAngleErr * ROLL_OUTER_P_GAIN - pFineGyro[0];
    pRoll->nP_ErrRate = pRoll->nCurrErrRate * ROLL_INNER_P_GAIN;
    pRoll->nI_ErrRate = _Clip3Float((pRoll->nI_ErrRate + (pRoll->nCurrErrRate * ROLL_INNER_I_GAIN) * nDiffTime), -100, 100);
    pRoll->nD_ErrRate = (pRoll->nCurrErrRate - pRoll->nPrevErrRate) * ROLL_INNER_D_GAIN / nDiffTime;
    pRoll->nBalance = pRoll->nP_ErrRate + pRoll->nI_ErrRate + pRoll->nD_ErrRate;

    //PITCH control
    pPitch->nAngleErr = pUsingRCVal[CH_TYPE_PITCH] - pFineRPY[1];
    pPitch->nCurrErrRate = pPitch->nAngleErr * PITCH_OUTER_P_GAIN - pFineGyro[1];
    pPitch->nP_ErrRate = pPitch->nCurrErrRate * PITCH_INNER_P_GAIN;
    pPitch->nI_ErrRate = _Clip3Float((pPitch->nI_ErrRate + (pPitch->nCurrErrRate * PITCH_INNER_I_GAIN) * nDiffTime), -100, 100);
    pPitch->nD_ErrRate = (pPitch->nCurrErrRate - pPitch->nPrevErrRate) * PITCH_INNER_D_GAIN / nDiffTime;
    pPitch->nBalance = pPitch->nP_ErrRate + pPitch->nI_ErrRate + pPitch->nD_ErrRate;

    //YAW control
    pYaw->nCurrErrRate = 0;//pUsingRCVal[CH_TYPE_YAW] + pFineGyro[2];// - pFineRPY[2];
    pYaw->nP_ErrRate = pYaw->nCurrErrRate * YAW_P_GAIN;
    pYaw->nI_ErrRate = _Clip3Float((pYaw->nI_ErrRate + pYaw->nCurrErrRate * YAW_I_GAIN * nDiffTime), -50, 50);
    pYaw->nTorque = pYaw->nP_ErrRate + pYaw->nI_ErrRate;

    // Backup for Next
    pRoll->nPrevErrRate = pRoll->nCurrErrRate;
    pPitch->nPrevErrRate = pPitch->nCurrErrRate;
    nPrevFineRPY[0] = pFineRPY[0];
    nPrevFineRPY[1] = pFineRPY[1];
    nPrevFineRPY[2] = pFineRPY[2];
}


inline void _CalculateThrottleVal()
{
    float                   nEstimatedThrottle = 0.0f;
    static float            nPrevEstimatedThrottle = 0.0f;
    AxisErrRate_T           *pPitch = &(pSelfFlyHndl->nPitch);
    AxisErrRate_T           *pRoll = &(pSelfFlyHndl->nRoll);
    AxisErrRate_T           *pYaw = &(pSelfFlyHndl->nYaw);
    unsigned long           *pThrottle = &(pSelfFlyHndl->nThrottle[0]);
    long                    *pUsingRCVal = &(pSelfFlyHndl->nUsingRCVal[0]);

    memset((void *)(&(pThrottle[0])), ESC_MIN, 4 * sizeof(int32_t));
    
    pUsingRCVal[CH_TYPE_THROTTLE] = floor(pUsingRCVal[CH_TYPE_THROTTLE] / ROUNDING_BASE) * ROUNDING_BASE;
    nEstimatedThrottle = (float)(map(pUsingRCVal[CH_TYPE_THROTTLE], RC_CH2_LOW, RC_CH2_HIGH, ESC_MIN, ESC_MAX));

    if((nEstimatedThrottle < ESC_MIN) || (nEstimatedThrottle > ESC_MAX))
        nEstimatedThrottle = nPrevEstimatedThrottle;

    nPrevEstimatedThrottle = nEstimatedThrottle;
    
    if(nEstimatedThrottle > ESC_TAKEOFF_OFFSET)
    {
        pThrottle[0] = _Clip3Int((( pPitch->nBalance + pRoll->nBalance) * 0.5 - pYaw->nTorque + nEstimatedThrottle), ESC_MIN, ESC_MAX);
        pThrottle[1] = _Clip3Int((( pPitch->nBalance - pRoll->nBalance) * 0.5 + pYaw->nTorque + nEstimatedThrottle), ESC_MIN, ESC_MAX);
        pThrottle[2] = _Clip3Int(((-pPitch->nBalance - pRoll->nBalance) * 0.5 - pYaw->nTorque + nEstimatedThrottle), ESC_MIN, ESC_MAX);
        pThrottle[3] = _Clip3Int(((-pPitch->nBalance + pRoll->nBalance) * 0.5 + pYaw->nTorque + nEstimatedThrottle), ESC_MIN, ESC_MAX);
    }
}


#endif /* PID_Controller_h */

