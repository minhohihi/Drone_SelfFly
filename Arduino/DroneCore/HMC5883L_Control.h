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

    nMagHndl = HMC5883L();
    pMagHndl = &nMagHndl;

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
    nDeclinationAngle = (7.0 + (59.76 / 60.0)) * DEG_TO_RAD_SCALE;

    Serialprintln(F(" Done"));

    // Reference WebSite
    // http://www.meccanismocomplesso.org/en/arduino-magnetic-magnetic-magnetometer-hmc5883l/
}


void _Mag_GetData()
{
    nMagHndl.getScaledHeading(&(nRawMag[X_AXIS]), &(nRawMag[Y_AXIS]), &(nRawMag[Z_AXIS]));

    // Calculate Heading
    //_Mag_CalculateDirection();
}


void _Mag_CalculateDirection()
{
    int                     i = 0;
    
    nMagHeadingRad = atan2(nRawMag[Y_AXIS], nRawMag[X_AXIS]);
    nMagHeadingRad -= nDeclinationAngle;      // If East, then Change Operation to PLUS

    if(nMagHeadingRad < 0)
        nMagHeadingRad += DOUBLE_RADIAN;

    if(nMagHeadingRad > DOUBLE_RADIAN)
        nMagHeadingRad -= DOUBLE_RADIAN;

    nMagHeadingDeg = nMagHeadingRad * RAD_TO_DEG_SCALE;

    if(nMagHeadingDeg >= 1 && nMagHeadingDeg < 240)
        nMagHeadingDeg = map(nMagHeadingDeg, 0, 239, 0, 179);
    else if(nMagHeadingDeg >= 240)
        nMagHeadingDeg = map(nMagHeadingDeg, 240, 360, 180, 360);

    // Smooth angles rotation for +/- 3deg
    nSmoothHeadingDegrees = round(nMagHeadingDeg);

    if((nSmoothHeadingDegrees < (nPrevHeadingDegrees + 3)) &&
       (nSmoothHeadingDegrees > (nPrevHeadingDegrees - 3)))
        nSmoothHeadingDegrees = nPrevHeadingDegrees;

    nPrevHeadingDegrees = nSmoothHeadingDegrees;
}

#endif /* HMC5883L_Controller_h */

