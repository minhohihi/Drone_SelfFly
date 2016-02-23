//
//  PID_Controller.h
//  SelfFly
//
//  Created by Maverick on 2016. 1. 21..
//

#ifndef __PID_CONTROL__
#define __PID_CONTROL__

void _Calculate_Altitude(float *pEstimatedThrottle);

inline void _CalculatePID()
{
    static long             nPrevRCVal[5] = {0, };              // Filter variables
    static float            nPrevFineRPY[3];
    #if !USE_NEW_PID
    AxisErrRate_T           *pPitch = &(pSelfFlyHndl->nPitch);
    AxisErrRate_T           *pRoll = &(pSelfFlyHndl->nRoll);
    AxisErrRate_T           *pYaw = &(pSelfFlyHndl->nYaw);
    #endif
    long                    *pUsingRCVal = &(pSelfFlyHndl->nUsingRCVal[0]);
    float                   *pFineGyro = &(pSelfFlyHndl->nFineGyro[0]);
    float                   *pFineRPY = &(pSelfFlyHndl->nFineRPY[0]);
    const float             nDiffTime = pSelfFlyHndl->nDiffTime;
    unsigned int            i = 0, j = 0;

    pSelfFlyHndl->nLock = true;
    memcpy(pUsingRCVal, &(pSelfFlyHndl->nCapturedRCVal[0]), MAX_CH_RC * sizeof(long));
    pSelfFlyHndl->nLock = false;

    #if USE_NEW_PID
    for(i=0 ; i<4 ; i++)
    {
        // Do Filter Except Throttle
        if(CH_TYPE_THROTTLE == i)
            continue;

        // Mapping RC Value to (-120) ~ (+120)  <== +-480 / 4
        if(pUsingRCVal[i] > 1520)
            pUsingRCVal[i] = (pUsingRCVal[i] - 1520) / 4;
        else if(pUsingRCVal[i] < 1480)
            pUsingRCVal[i] = (pUsingRCVal[i] - 1480) / 4;
        else
            pUsingRCVal[i] = 0;
    }
    #else
    pUsingRCVal[CH_TYPE_ROLL] = floor(pUsingRCVal[CH_TYPE_ROLL] / ROUNDING_BASE) * ROUNDING_BASE;
    pUsingRCVal[CH_TYPE_PITCH] = floor(pUsingRCVal[CH_TYPE_PITCH] / ROUNDING_BASE) * ROUNDING_BASE;
    pUsingRCVal[CH_TYPE_YAW] = floor(pUsingRCVal[CH_TYPE_YAW] / ROUNDING_BASE) * ROUNDING_BASE;

    pUsingRCVal[CH_TYPE_ROLL] = map(pUsingRCVal[CH_TYPE_ROLL], RC_CH0_LOW, RC_CH0_HIGH, ROLL_ANG_MIN, ROLL_ANG_MAX) - 1;
    pUsingRCVal[CH_TYPE_PITCH] = map(pUsingRCVal[CH_TYPE_PITCH], RC_CH1_LOW, RC_CH1_HIGH, PITCH_ANG_MIN, PITCH_ANG_MAX) - 1;
    pUsingRCVal[CH_TYPE_YAW] = map(pUsingRCVal[CH_TYPE_YAW], RC_CH3_LOW, RC_CH3_HIGH, YAW_RATE_MIN, YAW_RATE_MAX);

    if((pUsingRCVal[CH_TYPE_ROLL] < ROLL_ANG_MIN) || (pUsingRCVal[CH_TYPE_ROLL] > ROLL_ANG_MAX))
        pUsingRCVal[CH_TYPE_ROLL] = nPrevRCVal[CH_TYPE_ROLL];

    if((pUsingRCVal[CH_TYPE_PITCH] < PITCH_ANG_MIN) || (pUsingRCVal[CH_TYPE_PITCH] > PITCH_ANG_MAX))
        pUsingRCVal[CH_TYPE_PITCH] = nPrevRCVal[CH_TYPE_PITCH];

    if((pUsingRCVal[CH_TYPE_YAW] < YAW_RATE_MIN) || (pUsingRCVal[CH_TYPE_YAW] > YAW_RATE_MAX))
        pUsingRCVal[CH_TYPE_YAW] = nPrevRCVal[CH_TYPE_YAW];

    nPrevRCVal[CH_TYPE_ROLL] = pUsingRCVal[CH_TYPE_ROLL];
    nPrevRCVal[CH_TYPE_PITCH] = pUsingRCVal[CH_TYPE_PITCH];
    nPrevRCVal[CH_TYPE_YAW] = pUsingRCVal[CH_TYPE_YAW];
    #endif

    if(abs(pFineRPY[0] - nPrevFineRPY[0]) > 30)
        pFineRPY[0] = nPrevFineRPY[0];

    if(abs(pFineRPY[1] - nPrevFineRPY[1]) > 30)
        pFineRPY[1] = nPrevFineRPY[1];

    #if USE_NEW_PID
    {
        // PID configuration
        const float             nPIDGainTable[3][3] = {{1.4, 0.03, 15}, {1.4, 0.03, 15},
                                                       {4.0, 0.02, 0.0}};

        for(i=0 ; i<3 ; i++)
        {
            AxisErrRate_T       *pPIDCtrl = &(pSelfFlyHndl->nRPY_PID[i]);

            if(2 == i)
                j++;

            pPIDCtrl->nCurrErrRate = pUsingRCVal[j] - pFineRPY[j];
            pPIDCtrl->nP_ErrRate = nPIDGainTable[i][0] * pPIDCtrl->nCurrErrRate;
            pPIDCtrl->nI_ErrRate = _Clip3Float(pPIDCtrl->nI_ErrRate + (nPIDGainTable[i][1] * pPIDCtrl->nCurrErrRate), -400, 400);
            pPIDCtrl->nD_ErrRate = nPIDGainTable[i][2] * (pPIDCtrl->nCurrErrRate - pPIDCtrl->nPrevErrRate);
            pPIDCtrl->nBalance = _Clip3Float((pPIDCtrl->nP_ErrRate + pPIDCtrl->nI_ErrRate + pPIDCtrl->nD_ErrRate), -400, 400);

            pPIDCtrl->nPrevErrRate = pPIDCtrl->nCurrErrRate;

            j++;
        }
    }
    #else
    //ROLL control
    pRoll->nAngleErr = pUsingRCVal[CH_TYPE_ROLL] - pFineRPY[0];
    pRoll->nCurrErrRate = pRoll->nAngleErr * ROLL_OUTER_P_GAIN - pFineGyro[0];
    pRoll->nP_ErrRate = pRoll->nCurrErrRate * ROLL_INNER_P_GAIN;
    pRoll->nI_ErrRate = _Clip3Float((pRoll->nI_ErrRate + (pRoll->nCurrErrRate * ROLL_INNER_I_GAIN) * nDiffTime), -100, 100);
    pRoll->nD_ErrRate = (pRoll->nCurrErrRate - pRoll->nPrevErrRate) * ROLL_INNER_D_GAIN / nDiffTime;
    pRoll->nBalance = (pRoll->nP_ErrRate + pRoll->nI_ErrRate + pRoll->nD_ErrRate) * ((INVERSE_RC_ROLL) ? (-1) : (1));

    //PITCH control
    pPitch->nAngleErr = pUsingRCVal[CH_TYPE_PITCH] - pFineRPY[1];
    pPitch->nCurrErrRate = pPitch->nAngleErr * PITCH_OUTER_P_GAIN - pFineGyro[1];
    pPitch->nP_ErrRate = pPitch->nCurrErrRate * PITCH_INNER_P_GAIN;
    pPitch->nI_ErrRate = _Clip3Float((pPitch->nI_ErrRate + (pPitch->nCurrErrRate * PITCH_INNER_I_GAIN) * nDiffTime), -100, 100);
    pPitch->nD_ErrRate = (pPitch->nCurrErrRate - pPitch->nPrevErrRate) * PITCH_INNER_D_GAIN / nDiffTime;
    pPitch->nBalance = (pPitch->nP_ErrRate + pPitch->nI_ErrRate + pPitch->nD_ErrRate) * ((INVERSE_RC_PITCH) ? (-1) : (1));

    //YAW control
    pYaw->nCurrErrRate = pUsingRCVal[CH_TYPE_YAW];// + pFineGyro[2];// - pFineRPY[2];
    pYaw->nP_ErrRate = pYaw->nCurrErrRate * YAW_P_GAIN;
    pYaw->nI_ErrRate = _Clip3Float((pYaw->nI_ErrRate + pYaw->nCurrErrRate * YAW_I_GAIN * nDiffTime), -50, 50);
    pYaw->nTorque = (pYaw->nP_ErrRate + pYaw->nI_ErrRate) * ((INVERSE_RC_YAW) ? (-1) : (1));

    // Backup for Next
    pRoll->nPrevErrRate = pRoll->nCurrErrRate;
    pPitch->nPrevErrRate = pPitch->nCurrErrRate;
    nPrevFineRPY[0] = pFineRPY[0];
    nPrevFineRPY[1] = pFineRPY[1];
    nPrevFineRPY[2] = pFineRPY[2];
    #endif
}


inline void _CalculateThrottleVal()
{
    unsigned long           nEstimatedThrottle = 0;
    static unsigned long    nPrevEstimatedThrottle = 0;
    #if !USE_NEW_PID
    AxisErrRate_T           *pPitch = &(pSelfFlyHndl->nPitch);
    AxisErrRate_T           *pRoll = &(pSelfFlyHndl->nRoll);
    AxisErrRate_T           *pYaw = &(pSelfFlyHndl->nYaw);
    #endif
    unsigned long           *pThrottle = &(pSelfFlyHndl->nThrottle[0]);
    long                    *pUsingRCVal = &(pSelfFlyHndl->nUsingRCVal[0]);

    // Set Throttle Value as Min Value
    pThrottle[0] = ESC_MIN;
    pThrottle[1] = ESC_MIN;
    pThrottle[2] = ESC_MIN;
    pThrottle[3] = ESC_MIN;

    pUsingRCVal[CH_TYPE_THROTTLE] = floor(pUsingRCVal[CH_TYPE_THROTTLE] / ROUNDING_BASE) * ROUNDING_BASE;
    nEstimatedThrottle = map(pUsingRCVal[CH_TYPE_THROTTLE], RC_CH2_LOW, RC_CH2_HIGH, ESC_MIN, ESC_MAX);

    if((nEstimatedThrottle < ESC_MIN) || (nEstimatedThrottle > ESC_MAX))
        nEstimatedThrottle = nPrevEstimatedThrottle;

    nPrevEstimatedThrottle = nEstimatedThrottle;

    if((nEstimatedThrottle > ESC_TAKEOFF_OFFSET) && (DRONESTATUS_READY < pSelfFlyHndl->nDroneStatus))
    {
        #if USE_NEW_PID
        const float         nRollBalance = pSelfFlyHndl->nRPY_PID[0].nBalance;
        const float         nPitchBalance = pSelfFlyHndl->nRPY_PID[1].nBalance;
        const float         nYawBalance = pSelfFlyHndl->nRPY_PID[2].nBalance;

        pThrottle[0] = _Clip3Int((((int)( nRollBalance + nPitchBalance) * 0.5) - (int)(nYawBalance) + nEstimatedThrottle), ESC_ACTUAL_MIN, ESC_MAX);
        pThrottle[1] = _Clip3Int((((int)(-nRollBalance + nPitchBalance) * 0.5) + (int)(nYawBalance) + nEstimatedThrottle), ESC_ACTUAL_MIN, ESC_MAX);
        pThrottle[2] = _Clip3Int((((int)(-nRollBalance - nPitchBalance) * 0.5) - (int)(nYawBalance) + nEstimatedThrottle), ESC_ACTUAL_MIN, ESC_MAX);
        pThrottle[3] = _Clip3Int((((int)( nRollBalance - nPitchBalance) * 0.5) + (int)(nYawBalance) + nEstimatedThrottle), ESC_ACTUAL_MIN, ESC_MAX);
        #else
        pThrottle[0] = _Clip3Int((((int)( pPitch->nBalance + pRoll->nBalance) * 0.5) - (int)(pYaw->nTorque) + nEstimatedThrottle), ESC_ACTUAL_MIN, ESC_MAX);
        pThrottle[1] = _Clip3Int((((int)( pPitch->nBalance - pRoll->nBalance) * 0.5) + (int)(pYaw->nTorque) + nEstimatedThrottle), ESC_ACTUAL_MIN, ESC_MAX);
        pThrottle[2] = _Clip3Int((((int)(-pPitch->nBalance - pRoll->nBalance) * 0.5) - (int)(pYaw->nTorque) + nEstimatedThrottle), ESC_ACTUAL_MIN, ESC_MAX);
        pThrottle[3] = _Clip3Int((((int)(-pPitch->nBalance + pRoll->nBalance) * 0.5) + (int)(pYaw->nTorque) + nEstimatedThrottle), ESC_ACTUAL_MIN, ESC_MAX);
        #endif
    }
    else if(DRONESTATUS_READY >= pSelfFlyHndl->nDroneStatus)
    {
        // Set Throttle Value as Min Value
        pThrottle[0] = ESC_MIN;
        pThrottle[1] = ESC_MIN;
        pThrottle[2] = ESC_MIN;
        pThrottle[3] = ESC_MIN;
    }
}


void _Calculate_Altitude(float *pEstimatedThrottle)
{
    const long              *pUsingRCVal = &(pSelfFlyHndl->nUsingRCVal[0]);
    const float             nDistFromGnd = pSelfFlyHndl->SonicParam.nDistFromGnd;

    if(1500 < pUsingRCVal[CH_TYPE_TAKE_LAND])
    {
        //(nDistFromGnd - HOVERING_ALTITUDE)
    }
}

#endif /* PID_Controller_h */

