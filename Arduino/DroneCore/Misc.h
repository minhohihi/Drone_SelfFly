//
//  Misc.h
//  SelfFly
//
//  Created by Maverick on 2016. 1. 21..
//

#ifndef __MISC__
#define __MISC__


void _Check_Drone_Status()
{
    if(DRONESTATUS_STOP == pSelfFlyHndl->nDroneStatus)
    {
        int                 nLoopCnt = 0;
        
        do
        {
            // Should be Set Lowest Throttle & Highest Right Yaw for About 2 Sec. to Start Drone
            if((1100 >= pSelfFlyHndl->nCapturedRCVal[CH_TYPE_THROTTLE])
               && (1800 < pSelfFlyHndl->nCapturedRCVal[CH_TYPE_YAW]))
                nLoopCnt++;
            else
                nLoopCnt = 0;
            
            delay(50);
        }while(nLoopCnt < 40);
        
        pSelfFlyHndl->nDroneStatus = DRONESTATUS_READY;
        
        _LED_SetColor(0, 1, 0, 1);
        
        return;
    }
    else if(DRONESTATUS_STOP < pSelfFlyHndl->nDroneStatus)
    {
        static int      nLoopCnt = 0;
        
        if(DRONESTATUS_READY == pSelfFlyHndl->nDroneStatus)
        {
            if(1100 >= pSelfFlyHndl->nCapturedRCVal[CH_TYPE_THROTTLE])
                nLoopCnt++;
            else
            {
                nLoopCnt = 0;
                pSelfFlyHndl->nDroneStatus = DRONESTATUS_START;
                
                _LED_SetColor(0, 0, 1, 1);
            }
            
            if(nLoopCnt > DRONE_STOP_TIME_TH)
            {
                nLoopCnt = 0;
                pSelfFlyHndl->nDroneStatus = DRONESTATUS_STOP;
                
                _LED_SetColor(1, 0, 0, 1);
            }
        }
        else
        {
            if(1100 >= pSelfFlyHndl->nCapturedRCVal[CH_TYPE_THROTTLE])
            {
                nLoopCnt = 0;
                pSelfFlyHndl->nDroneStatus = DRONESTATUS_READY;
                
                _LED_SetColor(0, 1, 0, 1);
            }
        }
    }
}


void _GetSensorRawData()
{
    pSelfFlyHndl->nPrevSensorCapTime = pSelfFlyHndl->nCurrSensorCapTime;
    pSelfFlyHndl->nCurrSensorCapTime = micros();
    
    pSelfFlyHndl->nDiffTime = (pSelfFlyHndl->nCurrSensorCapTime - pSelfFlyHndl->nPrevSensorCapTime) / 1000000.0;
    pSelfFlyHndl->nSampleFreq = 1000000.0 / ((pSelfFlyHndl->nCurrSensorCapTime - pSelfFlyHndl->nPrevSensorCapTime));
    
    // Get AccelGyro Raw Data
    _AccelGyro_GetData();
    
    // Get Magnetic Raw Data
    _Mag_GetData();
    
    // Get Barometer Raw Data
    _Barometer_GetData();
    
    // Get Sonar Raw Data
    //_Sonar_GetData();
    _Sonar_GetData_WithPeriod();
}


void _Check_BatteryVolt()
{
    pSelfFlyHndl->nCurrBatteryVolt = (0.92 * pSelfFlyHndl->nCurrBatteryVolt) + (float)(analogRead(PIN_CHECK_POWER_STAT) + 65) * 0.09853;
    
    //Serialprintln(pSelfFlyHndl->nCurrBatteryVolt);
}


float _Clip3Float(const float nValue, const int MIN, const int MAX)
{
    float               nClipVal = nValue;
    
    if(nValue < MIN)
        nClipVal = MIN;
    else if(nValue > MAX)
        nClipVal = MAX;
    
    return nClipVal;
}


int _Clip3Int(const int nValue, const int MIN, const int MAX)
{
    int                 nClipVal = nValue;
    
    if(nValue < MIN)
        nClipVal = MIN;
    else if(nValue > MAX)
        nClipVal = MAX;
    
    return nClipVal;
}


/**
 * Fast inverse square root implementation
 * @see http://en.wikipedia.org/wiki/Fast_inverse_square_root
 */
float _InvSqrt(float nNumber)
{
    long                i = 0;
    float               x = 0.0f, y = 0.0f;
    const float         f = 1.5F;
    
    x = nNumber * 0.5F;
    y = nNumber;
    i = * ( long * ) &y;
    i = 0x5f375a86 - ( i >> 1 );
    y = * ( float * ) &i;
    y = y * ( f - ( x * y * y ) );
    
    return y;
}

#endif /* Misc_h */

