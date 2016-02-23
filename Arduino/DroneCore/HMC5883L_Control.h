//
//  HMC5883L_Controller.h
//  SelfFly
//
//  Created by Maverick on 2016. 1. 21..
//

#ifndef __HMC5883L_CONTROL__
#define __HMC5883L_CONTROL__

void _Mag_Initialize()
{
    HMC5883L            *pMagHndl = NULL;

    pSelfFlyHndl->nMagHndl = HMC5883L();
    pMagHndl = &(pSelfFlyHndl->nMagHndl);

    // initialize Magnetic
    Serialprintln(F(" Initializing Magnetic..."));
    pMagHndl->initialize();

    // Verify Vonnection
    Serialprint(F("    Testing device connections..."));
    Serialprintln(pMagHndl->testConnection() ? F("  HMC5883L connection successful") : F("  HMC5883L connection failed"));

    // Calibrate Magnetic
    //Serialprint(F("    Start Calibration of Magnetic Sensor (HMC5883L) "));
    //pMagHndl->calibrate();
    //pMagHndl->calibration_offset(1);
    //Serialprintln(F("Done"));

    pMagHndl->setMode(HMC5883L_MODE_CONTINUOUS);
    pMagHndl->setGain(HMC5883L_GAIN_1090);
    pMagHndl->setDataRate(HMC5883L_RATE_75);
    pMagHndl->setSampleAveraging(HMC5883L_AVERAGING_8);

    // Date: 2015-11-05
    // Location: Seoul, South Korea
    // Latitude: 37.0000° North
    // Longitude: 126.0000° East
    // Magnetic declination: 7° 59.76' West
    // Annual Change (minutes/year): 3.9 '/y West
    // http://www.geomag.nrcan.gc.ca/calc/mdcal-en.php
    // http://www.magnetic-declination.com/
    pSelfFlyHndl->nMagParam.nDeclinationAngle = (7.0 + (59.76 / 60.0)) * DEG_TO_RAD_SCALE;

    Serialprintln(F(" Done"));

    // Reference WebSite
    // http://www.meccanismocomplesso.org/en/arduino-magnetic-magnetic-magnetometer-hmc5883l/
}


void _Mag_GetData()
{
    float                   *pRawMag = &(pSelfFlyHndl->nMagParam.nRawMag[X_AXIS]);

    pSelfFlyHndl->nMagHndl.getScaledHeading(&(pRawMag[X_AXIS]), &(pRawMag[Y_AXIS]), &(pRawMag[Z_AXIS]));

    // Calculate Heading
    //_Mag_CalculateDirection();
}


void _Mag_CalculateDirection()
{
    int                     i = 0;
    MagneticParam_T         *pMagParam = &(pSelfFlyHndl->nMagParam);

    pMagParam->nMagHeadingRad = atan2(pMagParam->nRawMag[Y_AXIS], pMagParam->nRawMag[X_AXIS]);
    pMagParam->nMagHeadingRad -= pMagParam->nDeclinationAngle;      // If East, then Change Operation to PLUS

    if(pMagParam->nMagHeadingRad < 0)
        pMagParam->nMagHeadingRad += DOUBLE_RADIAN;

    if(pMagParam->nMagHeadingRad > DOUBLE_RADIAN)
        pMagParam->nMagHeadingRad -= DOUBLE_RADIAN;

    pMagParam->nMagHeadingDeg = pMagParam->nMagHeadingRad * RAD_TO_DEG_SCALE;

    if(pMagParam->nMagHeadingDeg >= 1 && pMagParam->nMagHeadingDeg < 240)
        pMagParam->nMagHeadingDeg = map(pMagParam->nMagHeadingDeg, 0, 239, 0, 179);
    else if(pMagParam->nMagHeadingDeg >= 240)
        pMagParam->nMagHeadingDeg = map(pMagParam->nMagHeadingDeg, 240, 360, 180, 360);

    // Smooth angles rotation for +/- 3deg
    pMagParam->nSmoothHeadingDegrees = round(pMagParam->nMagHeadingDeg);

    if((pMagParam->nSmoothHeadingDegrees < (pMagParam->nPrevHeadingDegrees + 3)) &&
       (pMagParam->nSmoothHeadingDegrees > (pMagParam->nPrevHeadingDegrees - 3)))
        pMagParam->nSmoothHeadingDegrees = pMagParam->nPrevHeadingDegrees;

    pMagParam->nPrevHeadingDegrees = pMagParam->nSmoothHeadingDegrees;
}

#endif /* HMC5883L_Controller_h */

