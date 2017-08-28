//
//  PID_Controller.h
//  SelfFly
//
//  Created by Maverick on 2016. 1. 21..
//

#ifndef __PID_CONTROL__
#define __PID_CONTROL__

float             nPIDGainTable[3][4] = {{1.52, 0.01, 15.40, 400.0},     // Roll's P, I, D
                                         {1.52, 0.01, 15.40, 400.0},     // Pitch's P, I, D
                                         {4.00, 0.02, 00.00, 400.0}};    // Yaw's P, I, D
        
void _Calculate_Altitude(float *pEstimatedThrottle);

void _CalculatePID()
{
    float                   nEstimatedRCVal[CH_TYPE_MAX] = {0.0, };
    float                   nCurrErrRate;
    int                     i = 0, j = 0;

    #if USE_EXT_SR_READ
    {
        if(Serial.available())
        {
            char ch = Serial.read();
            
            if(ch == 'a')
            {
                nPIDGainTable[0][0] -= 0.01;
                nPIDGainTable[1][0] -= 0.01;
            }
            else if(ch == 'z')
            {
                nPIDGainTable[0][0] += 0.01;
                nPIDGainTable[1][0] += 0.01;
            }
            else if(ch == 's')
            {
                nPIDGainTable[0][1] -= 0.01;
                nPIDGainTable[1][1] -= 0.01;
            }
            else if(ch == 'x')
            {
                nPIDGainTable[0][1] += 0.01;
                nPIDGainTable[1][1] += 0.01;
            }
            else if(ch == 'd')
            {
                nPIDGainTable[0][2] -= 0.01;
                nPIDGainTable[1][2] -= 0.01;
            }
            else if(ch == 'c')
            {
                nPIDGainTable[0][2] += 0.01;
                nPIDGainTable[1][2] += 0.01;
            }
        }
    }
    #endif
    
    if(_gCompensatedRCVal[_gRCChMap[CH_TYPE_ROLL]] > 1508)
        nEstimatedRCVal[CH_TYPE_ROLL] = (_gCompensatedRCVal[_gRCChMap[CH_TYPE_ROLL]] - 1508);
    else if(_gCompensatedRCVal[_gRCChMap[CH_TYPE_ROLL]] < 1492)
        nEstimatedRCVal[CH_TYPE_ROLL] = (_gCompensatedRCVal[_gRCChMap[CH_TYPE_ROLL]] - 1492);
    
    nEstimatedRCVal[CH_TYPE_ROLL] -= _gRollLevelAdjust;
    nEstimatedRCVal[CH_TYPE_ROLL] /= 3.0;

    if(_gCompensatedRCVal[_gRCChMap[CH_TYPE_PITCH]] > 1508)
        nEstimatedRCVal[CH_TYPE_PITCH] = (_gCompensatedRCVal[_gRCChMap[CH_TYPE_PITCH]] - 1508);
    else if(_gCompensatedRCVal[_gRCChMap[CH_TYPE_PITCH]] < 1492)
        nEstimatedRCVal[CH_TYPE_PITCH] = (_gCompensatedRCVal[_gRCChMap[CH_TYPE_PITCH]] - 1492);

    nEstimatedRCVal[CH_TYPE_PITCH] -= _gPitchLevelAdjust;
    nEstimatedRCVal[CH_TYPE_PITCH] /= 3.0;

    if(_gCompensatedRCVal[_gRCChMap[CH_TYPE_THROTTLE]] > 1050)
    {
        if(_gCompensatedRCVal[_gRCChMap[CH_TYPE_YAW]] > 1508)
            nEstimatedRCVal[CH_TYPE_YAW] = (_gCompensatedRCVal[_gRCChMap[CH_TYPE_YAW]] - 1508) / 3.0;
        else if(_gCompensatedRCVal[_gRCChMap[CH_TYPE_YAW]] < 1492)
            nEstimatedRCVal[CH_TYPE_YAW] = (_gCompensatedRCVal[_gRCChMap[CH_TYPE_YAW]] - 1492) / 3.0;
    }

    // PID configuration
    {
        // Roll
        nCurrErrRate = _gEstRoll - nEstimatedRCVal[CH_TYPE_ROLL];
        
        _gRPY_PID[0].nP_ErrRate = nPIDGainTable[0][0] * nCurrErrRate;
        _gRPY_PID[0].nI_ErrRate += nPIDGainTable[0][1] * nCurrErrRate;
        _gRPY_PID[0].nI_ErrRate = _Clip3Float(_gRPY_PID[0].nI_ErrRate, -nPIDGainTable[0][3], nPIDGainTable[0][3]);
        _gRPY_PID[0].nD_ErrRate = nPIDGainTable[0][2] * (nCurrErrRate - _gRPY_PID[0].nPrevErrRate);
        
        _gRPY_PID[0].nBalance   = _gRPY_PID[0].nP_ErrRate + _gRPY_PID[0].nI_ErrRate + _gRPY_PID[0].nD_ErrRate;
        _gRPY_PID[0].nBalance   = _Clip3Float(_gRPY_PID[0].nBalance, -nPIDGainTable[0][3], nPIDGainTable[0][3]);
        
        _gRPY_PID[0].nPrevErrRate = nCurrErrRate;
    }

    {
        // Picth
        nCurrErrRate = _gEstPitch - nEstimatedRCVal[CH_TYPE_PITCH];
        
        _gRPY_PID[1].nP_ErrRate = nPIDGainTable[1][0] * nCurrErrRate;
        _gRPY_PID[1].nI_ErrRate += nPIDGainTable[1][1] * nCurrErrRate;
        _gRPY_PID[1].nI_ErrRate = _Clip3Float(_gRPY_PID[1].nI_ErrRate, -nPIDGainTable[1][3], nPIDGainTable[1][3]);
        _gRPY_PID[1].nD_ErrRate = nPIDGainTable[1][2] * (nCurrErrRate - _gRPY_PID[1].nPrevErrRate);
        
        _gRPY_PID[1].nBalance   = _gRPY_PID[1].nP_ErrRate + _gRPY_PID[1].nI_ErrRate + _gRPY_PID[1].nD_ErrRate;
        _gRPY_PID[1].nBalance   = _Clip3Float(_gRPY_PID[1].nBalance, -nPIDGainTable[1][3], nPIDGainTable[1][3]);
        
        _gRPY_PID[1].nPrevErrRate = nCurrErrRate;
    }

    {
        // Yaw
        nCurrErrRate = _gEstYaw - nEstimatedRCVal[CH_TYPE_YAW];
        
        _gRPY_PID[2].nP_ErrRate = nPIDGainTable[2][0] * nCurrErrRate;
        _gRPY_PID[2].nI_ErrRate += nPIDGainTable[2][1] * nCurrErrRate;
        _gRPY_PID[2].nI_ErrRate = _Clip3Float(_gRPY_PID[2].nI_ErrRate, -nPIDGainTable[2][3], nPIDGainTable[2][3]);
        _gRPY_PID[2].nD_ErrRate = nPIDGainTable[2][2] * (nCurrErrRate - _gRPY_PID[2].nPrevErrRate);
        
        _gRPY_PID[2].nBalance   = _gRPY_PID[2].nP_ErrRate + _gRPY_PID[2].nI_ErrRate + _gRPY_PID[2].nD_ErrRate;
        _gRPY_PID[2].nBalance   = _Clip3Float(_gRPY_PID[2].nBalance, -nPIDGainTable[2][3], nPIDGainTable[2][3]);
        
        _gRPY_PID[2].nPrevErrRate = nCurrErrRate;
    }
}


void _CalculateThrottleVal()
{
    int                     nThrottle = _gCompensatedRCVal[_gRCChMap[CH_TYPE_THROTTLE]];
    int                     i = 0;
    
    if(DRONESTATUS_START == _gDroneStatus)
    {
        int                 nRollBalance    = (int)(_gRPY_PID[0].nBalance);
        int                 nPitchBalance   = (int)(_gRPY_PID[1].nBalance);
        int                 nYawBalance     = (int)(_gRPY_PID[2].nBalance);

        if(1800 < nThrottle)
            nThrottle = 1800;
        
        _gESCOutput[0] = nThrottle + nRollBalance + nPitchBalance + nYawBalance;
        _gESCOutput[1] = nThrottle - nRollBalance + nPitchBalance - nYawBalance;
        _gESCOutput[2] = nThrottle - nRollBalance - nPitchBalance + nYawBalance;
        _gESCOutput[3] = nThrottle + nRollBalance - nPitchBalance - nYawBalance;

        for(i=0 ; i<4 ; i++)
            _gESCOutput[i] = _Clip3Float(_gESCOutput[i], ESC_ACTUAL_MIN, ESC_ACTUAL_MAX);
    }
    else
    {
        // Set Throttle Value as Min Value
        for(i=0 ; i<4 ; i++)
            _gESCOutput[i] = ESC_MIN;
    }
}


void _Calculate_Altitude(float *pEstimatedThrottle)
{
    if(1500 < _gCompensatedRCVal[_gRCChMap[CH_TYPE_TAKE_LAND]])
    {
        //(_gDistFromGnd - HOVERING_ALTITUDE)
    }
}

#endif /* PID_Controller_h */





