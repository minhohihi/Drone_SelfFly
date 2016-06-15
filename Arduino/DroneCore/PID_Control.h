//
//  PID_Controller.h
//  SelfFly
//
//  Created by Maverick on 2016. 1. 21..
//

#ifndef __PID_CONTROL__
#define __PID_CONTROL__

float             nPIDGainTable[3][4] = {{1.52, 0.01, 15.40, 400},     // Roll's P, I, D
                                         {1.52, 0.01, 15.40, 400},     // Pitch's P, I, D
                                         {4.00, 0.02, 00.00, 400}};    // Yaw's P, I, D
        
void _Calculate_Altitude(float *pEstimatedThrottle);

void _CalculatePID()
{
    float                   nEstimatedRCVal[CH_TYPE_MAX];
    float                   nCurrErrRate;
    int                     i = 0, j = 0;
    
    nEstimatedRCVal[CH_TYPE_ROLL] = 0;
    if(nCompensatedRCVal[CH_TYPE_ROLL] > 1508)
        nEstimatedRCVal[CH_TYPE_ROLL] = (nCompensatedRCVal[CH_TYPE_ROLL] - 1508) / 3.0;
    else if(nCompensatedRCVal[CH_TYPE_ROLL] < 1492)
        nEstimatedRCVal[CH_TYPE_ROLL] = (nCompensatedRCVal[CH_TYPE_ROLL] - 1492) / 3.0;
    
    nEstimatedRCVal[CH_TYPE_PITCH] = 0;
    if(nCompensatedRCVal[CH_TYPE_PITCH] > 1508)
        nEstimatedRCVal[CH_TYPE_PITCH] = (nCompensatedRCVal[CH_TYPE_PITCH] - 1508) / 3.0;
    else if(nCompensatedRCVal[CH_TYPE_PITCH] < 1492)
        nEstimatedRCVal[CH_TYPE_PITCH] = (nCompensatedRCVal[CH_TYPE_PITCH] - 1492) / 3.0;
    
    nEstimatedRCVal[CH_TYPE_YAW] = 0;
    if(nCompensatedRCVal[CH_TYPE_THROTTLE] > 1050)
    {
        if(nCompensatedRCVal[CH_TYPE_YAW] > 1508)
            nEstimatedRCVal[CH_TYPE_YAW] = (nCompensatedRCVal[CH_TYPE_YAW] - 1508) / 3.0;
        else if(nCompensatedRCVal[CH_TYPE_YAW] < 1492)
            nEstimatedRCVal[CH_TYPE_YAW] = (nCompensatedRCVal[CH_TYPE_YAW] - 1492) / 3.0;
    }

    // PID configuration
    // Roll
    {
        nCurrErrRate = nEstimatedRPY[1] - nEstimatedRCVal[CH_TYPE_ROLL];
        
        nRPY_PID[0].nP_ErrRate = nPIDGainTable[0][0] * nCurrErrRate;
        nRPY_PID[0].nI_ErrRate += nPIDGainTable[0][1] * nCurrErrRate;
        nRPY_PID[0].nI_ErrRate = _Clip3Float(nRPY_PID[0].nI_ErrRate, -nPIDGainTable[0][3], nPIDGainTable[0][3]);
        nRPY_PID[0].nD_ErrRate = nPIDGainTable[0][2] * (nCurrErrRate - nRPY_PID[0].nPrevErrRate);
        
        nRPY_PID[0].nBalance = nRPY_PID[0].nP_ErrRate + nRPY_PID[0].nI_ErrRate + nRPY_PID[0].nD_ErrRate;
        nRPY_PID[0].nBalance = _Clip3Float(nRPY_PID[0].nBalance, -nPIDGainTable[0][3], nPIDGainTable[0][3]);
        
        nRPY_PID[0].nPrevErrRate = nCurrErrRate;
    }

    // Picth
    {
        nCurrErrRate = nEstimatedRPY[0] - nEstimatedRCVal[CH_TYPE_PITCH];
        
        nRPY_PID[1].nP_ErrRate = nPIDGainTable[1][0] * nCurrErrRate;
        nRPY_PID[1].nI_ErrRate += nPIDGainTable[1][1] * nCurrErrRate;
        nRPY_PID[1].nI_ErrRate = _Clip3Float(nRPY_PID[1].nI_ErrRate, -nPIDGainTable[1][3], nPIDGainTable[1][3]);
        nRPY_PID[1].nD_ErrRate = nPIDGainTable[1][2] * (nCurrErrRate - nRPY_PID[1].nPrevErrRate);
        
        nRPY_PID[1].nBalance = nRPY_PID[1].nP_ErrRate + nRPY_PID[1].nI_ErrRate + nRPY_PID[1].nD_ErrRate;
        nRPY_PID[1].nBalance = _Clip3Float(nRPY_PID[1].nBalance, -nPIDGainTable[1][3], nPIDGainTable[1][3]);
        
        nRPY_PID[1].nPrevErrRate = nCurrErrRate;
    }

    // Yaw
    {
        nCurrErrRate = nEstimatedRPY[2] - nEstimatedRCVal[CH_TYPE_YAW];
        
        nRPY_PID[2].nP_ErrRate = nPIDGainTable[2][0] * nCurrErrRate;
        nRPY_PID[2].nI_ErrRate += nPIDGainTable[2][1] * nCurrErrRate;
        nRPY_PID[2].nI_ErrRate = _Clip3Float(nRPY_PID[2].nI_ErrRate, -nPIDGainTable[2][3], nPIDGainTable[2][3]);
        nRPY_PID[2].nD_ErrRate = nPIDGainTable[2][2] * (nCurrErrRate - nRPY_PID[2].nPrevErrRate);
        
        nRPY_PID[2].nBalance = nRPY_PID[2].nP_ErrRate + nRPY_PID[2].nI_ErrRate + nRPY_PID[2].nD_ErrRate;
        nRPY_PID[2].nBalance = _Clip3Float(nRPY_PID[2].nBalance, -nPIDGainTable[2][3], nPIDGainTable[2][3]);
        
        nRPY_PID[2].nPrevErrRate = nCurrErrRate;
    }
}


void _CalculateThrottleVal()
{
    int                     nThrottle = nCompensatedRCVal[CH_TYPE_THROTTLE];
    int                     i = 0;
    
    if(DRONESTATUS_START == nDroneStatus)
    {
        int                 nRollBalance    = (int)(nRPY_PID[0].nBalance);
        int                 nPitchBalance   = (int)(nRPY_PID[1].nBalance);
        int                 nYawBalance     = (int)(nRPY_PID[2].nBalance);

        if(1800 < nThrottle)
            nThrottle = 1800;
        
        nESCOutput[0] = nThrottle + nRollBalance + nPitchBalance + nYawBalance;
        nESCOutput[1] = nThrottle - nRollBalance + nPitchBalance - nYawBalance;
        nESCOutput[2] = nThrottle - nRollBalance - nPitchBalance + nYawBalance;
        nESCOutput[3] = nThrottle + nRollBalance - nPitchBalance - nYawBalance;

        for(i=0 ; i<4 ; i++)
        {
            if(nESCOutput[i] < ESC_ACTUAL_MIN)
                nESCOutput[i] = ESC_ACTUAL_MIN;

            if(nESCOutput[i] > ESC_ACTUAL_MAX)
                nESCOutput[i] = ESC_ACTUAL_MAX;
        }
    }
    else
    {
        // Set Throttle Value as Min Value
        for(i=0 ; i<4 ; i++)
            nESCOutput[i] = ESC_MIN;
    }
}


void _Calculate_Altitude(float *pEstimatedThrottle)
{
    if(1500 < nCompensatedRCVal[CH_TYPE_TAKE_LAND])
    {
        //(nDistFromGnd - HOVERING_ALTITUDE)
    }
}

#endif /* PID_Controller_h */

