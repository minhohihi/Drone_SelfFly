//
//  HS5611_Controller.h
//  SelfFly
//
//  Created by Maverick on 2016. 1. 21..
//

#ifndef __HS5611_CONTROL__
#define __HS5611_CONTROL__

void _Barometer_GetData();
void _Barometer_CalculateData();

void _Barometer_Initialize()
{
    int                     i = 0;
    BaroParam_T             *pBaroParam = &(pSelfFlyHndl->nBaroParam);
    MS561101BA              *pBaroHndl = &(pSelfFlyHndl->nBaroHndl);

    Serialprint(F(" Initializing Barometer Sensor (MS5611)..."));

    pBaroHndl->init(MS561101BA_ADDR_CSB_LOW);

    for(i=0 ; i<50 ; i++)
    {
        _Barometer_GetData();
        delay(20);
    }

    // Get Average Pressure & Temperature
    pBaroParam->nAvgTemp = pBaroHndl->getAvgTemp();
    pBaroParam->nAvgPressure = pBaroHndl->getAvgPressure();

    // Get Reference Altitude
    pBaroParam->nRefAbsoluteAltitude = pBaroHndl->getAltitude(pBaroParam->nAvgPressure, pBaroParam->nAvgTemp);

    Serialprintln(F(" Done"));
}


void _Barometer_GetData()
{
    BaroParam_T             *pBaroParam = &(pSelfFlyHndl->nBaroParam);
    MS561101BA              *pBaroHndl = &(pSelfFlyHndl->nBaroHndl);

    pBaroParam->nRawTemp = pBaroHndl->getTemperature(MS561101BA_OSR_512);
    pBaroParam->nRawPressure = pBaroHndl->getPressure(MS561101BA_OSR_512);

    // Push to Array to Get Average Pressure & Temperature
    pBaroHndl->pushTemp(pBaroParam->nRawTemp);
    pBaroHndl->pushPressure(pBaroParam->nRawPressure);
    
    // Calculate Altitude
    _Barometer_CalculateData();
}


void _Barometer_CalculateData()
{
    BaroParam_T             *pBaroParam = &(pSelfFlyHndl->nBaroParam);
    MS561101BA              *pBaroHndl = &(pSelfFlyHndl->nBaroHndl);

    // Get Average Pressure & Temperature
    pBaroParam->nAvgTemp = pBaroHndl->getAvgTemp();
    pBaroParam->nAvgPressure = pBaroHndl->getAvgPressure();

    // Get Altitude
    pBaroParam->nRawAbsoluteAltitude = pBaroHndl->getAltitude(pBaroParam->nAvgPressure, pBaroParam->nAvgTemp);

    // Push to Array to Get Average Altitude
    pBaroHndl->pushAltitude(pBaroParam->nRawAbsoluteAltitude);

    // Get Average Pressure & Temperature
    pBaroParam->nAvgAbsoluteAltitude = pBaroHndl->getAvgAltitude();

    // Get Vertical Speed
    pBaroParam->nVerticalSpeed = abs(pBaroParam->nAvgAbsoluteAltitude - pBaroParam->nPrevAvgAbsoluteAltitude) / (pSelfFlyHndl->nDiffTime);
    pBaroParam->nRelativeAltitude = pBaroParam->nAvgAbsoluteAltitude - pBaroParam->nRefAbsoluteAltitude;

    pBaroParam->nPrevAvgAbsoluteAltitude = pBaroParam->nAvgAbsoluteAltitude;
}
#endif /* HS5611_Controller_h */
