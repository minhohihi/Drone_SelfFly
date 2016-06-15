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
    MS561101BA              *pBaroHndl = &(nBaroHndl);

    Serialprint(F(" Initializing Barometer Sensor (MS5611)..."));

    pBaroHndl->init(MS561101BA_ADDR_CSB_LOW);

    for(i=0 ; i<50 ; i++)
    {
        _Barometer_GetData();
        delay(20);
    }

    // Get Average Pressure & Temperature
    nAvgTemp = pBaroHndl->getAvgTemp();
    nAvgPressure = pBaroHndl->getAvgPressure();

    // Get Reference Altitude
    nRefAbsoluteAltitude = pBaroHndl->getAltitude(nAvgPressure, nAvgTemp);

    Serialprintln(F(" Done"));
}


void _Barometer_GetData()
{
    MS561101BA              *pBaroHndl = &(nBaroHndl);

    nRawTemp = pBaroHndl->getTemperature(MS561101BA_OSR_512);
    nRawPressure = pBaroHndl->getPressure(MS561101BA_OSR_512);

    // Push to Array to Get Average Pressure & Temperature
    pBaroHndl->pushTemp(nRawTemp);
    pBaroHndl->pushPressure(nRawPressure);

    // Calculate Altitude
    _Barometer_CalculateData();
}


void _Barometer_CalculateData()
{
    MS561101BA              *pBaroHndl = &(nBaroHndl);

    // Get Average Pressure & Temperature
    nAvgTemp = pBaroHndl->getAvgTemp();
    nAvgPressure = pBaroHndl->getAvgPressure();

    // Get Altitude
    nRawAbsoluteAltitude = pBaroHndl->getAltitude(nAvgPressure, nAvgTemp);

    // Push to Array to Get Average Altitude
    pBaroHndl->pushAltitude(nRawAbsoluteAltitude);

    // Get Average Pressure & Temperature
    nAvgAbsoluteAltitude = pBaroHndl->getAvgAltitude();

    // Get Vertical Speed
    nVerticalSpeed = abs(nAvgAbsoluteAltitude - nPrevAvgAbsoluteAltitude) / (double)(nDiffTime);
    nRelativeAltitude = nAvgAbsoluteAltitude - nRefAbsoluteAltitude;

    nPrevAvgAbsoluteAltitude = nAvgAbsoluteAltitude;
}
#endif /* HS5611_Controller_h */

