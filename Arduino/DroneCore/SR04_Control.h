//
//  SR04_Controller.h
//  SelfFly
//
//  Created by Maverick on 2016. 1. 21..
//

#ifndef __SR04_CONTROL__
#define __SR04_CONTROL__

void _Sonar_GetData();

void _Sonar_Initialize()
{
    int                     i = 0;

    // Set A1 as Digital Output Mode for Sonar Sensor (HC-SR04)
    DDRC |= B00000010;

    // Calibrate Sonar Sensor
    for(i=0 ; i<50 ; i++)
    {
        _Sonar_GetData();
        delay(20);
    }
}


void _Sonar_GetData()
{
    PORTC |= B00000010;         
    delayMicroseconds(10);
    PORTC &= B11111101;         

    // Get Raw Distance Value
    pSelfFlyHndl->SonicParam.nRawDist = pulseIn(PIN_SONAR_ECHO, HIGH, SONAR_MAX_WAIT);

    // Calculate Distance From Ground
    pSelfFlyHndl->SonicParam.nDistFromGnd = pSelfFlyHndl->SonicParam.nRawDist * 0.017; // (340(m/s) * 1000(mm) / 1000000(microsec) / 2(oneway))
}


void _Sonar_GetData_WithPeriod()
{
    static unsigned long    nPrevTime = 0;
    unsigned long           nCurrTime = micros();
    
    if((nCurrTime - nPrevTime) > SONAR_GETDATA_PERIOD)
    {
        _Sonar_GetData();
        
        nPrevTime = nCurrTime;
    }
}

#endif /* SR04_Controller_h */

