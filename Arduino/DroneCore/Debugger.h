//
//  Debugger.h
//  SelfFly
//
//  Created by Maverick on 2016. 1. 21..
//

#ifndef __DEBUGGER__
#define __DEBUGGER__

#if __PRINT_DEBUG__
void _print_CaturedRC_Signals()
{
    long                    *pCapturedRCVal = &(pSelfFlyHndl->nCapturedRCVal[0]);

    Serialprint("   //   RC_Roll:");
    if(pCapturedRCVal[CH_TYPE_ROLL] - 1480 < 0)Serialprint("<<<");
    else if(pCapturedRCVal[CH_TYPE_ROLL] - 1520 > 0)Serialprint(">>>");
    else Serialprint("-+-");
    Serialprint(pCapturedRCVal[CH_TYPE_ROLL]);

    Serialprint("   RC_Pitch:");
    if(pCapturedRCVal[CH_TYPE_PITCH] - 1480 < 0)Serialprint("^^^");
    else if(pCapturedRCVal[CH_TYPE_PITCH] - 1520 > 0)Serialprint("vvv");
    else Serialprint("-+-");
    Serialprint(pCapturedRCVal[CH_TYPE_PITCH]);

    Serialprint("   RC_Throttle:");
    if(pCapturedRCVal[CH_TYPE_THROTTLE] - 1480 < 0)Serialprint("vvv");
    else if(pCapturedRCVal[CH_TYPE_THROTTLE] - 1520 > 0)Serialprint("^^^");
    else Serialprint("-+-");
    Serialprint(pCapturedRCVal[CH_TYPE_THROTTLE]);

    Serialprint("   RC_Yaw:");
    if(pCapturedRCVal[CH_TYPE_YAW] - 1480 < 0)Serialprint("<<<");
    else if(pCapturedRCVal[CH_TYPE_YAW] - 1520 > 0)Serialprint(">>>");
    else Serialprint("-+-");
    Serialprint(pCapturedRCVal[CH_TYPE_YAW]);

    Serialprint("   RC_Gear:");
    if(pCapturedRCVal[CH_TYPE_TAKE_LAND] - 1480 < 0)Serialprint("<<<");
    else if(pCapturedRCVal[CH_TYPE_TAKE_LAND] - 1520 > 0)Serialprint(">>>");
    else Serialprint("-+-");
    Serialprint(pCapturedRCVal[CH_TYPE_TAKE_LAND]);
}


void _print_UsingRC_Signals()
{
    long                    *pUsingRCVal = &(pSelfFlyHndl->nUsingRCVal[0]);
    
    Serialprint("   //   RC_Roll:");
    if(pUsingRCVal[CH_TYPE_ROLL] - 1480 < 0)Serialprint("<<<");
    else if(pUsingRCVal[CH_TYPE_ROLL] - 1520 > 0)Serialprint(">>>");
    else Serialprint("-+-");
    Serialprint(pUsingRCVal[CH_TYPE_ROLL]);
    
    Serialprint("   RC_Pitch:");
    if(pUsingRCVal[CH_TYPE_PITCH] - 1480 < 0)Serialprint("^^^");
    else if(pUsingRCVal[CH_TYPE_PITCH] - 1520 > 0)Serialprint("vvv");
    else Serialprint("-+-");
    Serialprint(pUsingRCVal[CH_TYPE_PITCH]);
    
    Serialprint("   RC_Throttle:");
    if(pUsingRCVal[CH_TYPE_THROTTLE] - 1480 < 0)Serialprint("vvv");
    else if(pUsingRCVal[CH_TYPE_THROTTLE] - 1520 > 0)Serialprint("^^^");
    else Serialprint("-+-");
    Serialprint(pUsingRCVal[CH_TYPE_THROTTLE]);
    
    Serialprint("   RC_Yaw:");
    if(pUsingRCVal[CH_TYPE_YAW] - 1480 < 0)Serialprint("<<<");
    else if(pUsingRCVal[CH_TYPE_YAW] - 1520 > 0)Serialprint(">>>");
    else Serialprint("-+-");
    Serialprint(pUsingRCVal[CH_TYPE_YAW]);
    
    Serialprint("   RC_Gear:");
    if(pUsingRCVal[CH_TYPE_TAKE_LAND] - 1480 < 0)Serialprint("<<<");
    else if(pUsingRCVal[CH_TYPE_TAKE_LAND] - 1520 > 0)Serialprint(">>>");
    else Serialprint("-+-");
    Serialprint(pUsingRCVal[CH_TYPE_TAKE_LAND]);
}


void _print_Gyro_Signals()
{
    float                   *pFineAngle = &(pSelfFlyHndl->nAccelGyroParam.nFineAngle[0]);

    Serialprint("   Gx: ");
    Serialprint(pSelfFlyHndl->nAccelGyroParam.nRawGyro[0]);
    Serialprint("   Gy: ");
    Serialprint(pSelfFlyHndl->nAccelGyroParam.nRawGyro[1]);
    Serialprint("   Gz: ");
    Serialprint(pSelfFlyHndl->nAccelGyroParam.nRawGyro[2]);

    Serialprint("   Ax: ");
    Serialprint(pSelfFlyHndl->nAccelGyroParam.nRawAccel[0]);
    Serialprint("   Ay: ");
    Serialprint(pSelfFlyHndl->nAccelGyroParam.nRawAccel[1]);
    Serialprint("   Az: ");
    Serialprint(pSelfFlyHndl->nAccelGyroParam.nRawAccel[2]);

    Serialprint("   Temp : ");
    Serialprint(pSelfFlyHndl->nAccelGyroParam.nRawTemp/340.00 + 36.53);
}


void _print_Throttle_Signals()
{
     unsigned long  *pThrottle = &(pSelfFlyHndl->nThrottle[0]);

    Serialprint("   //    Thrt1 : ");
    Serialprint(pThrottle[0]);
    Serialprint("  Thrt2 : ");
    Serialprint(pThrottle[1]);
    Serialprint("  Thrt3 : ");
    Serialprint(pThrottle[2]);
    Serialprint("  Thrt4 : ");
    Serialprint(pThrottle[3]);
}


void _print_RPY_Signals()
{
     float          *pFineRPY = &(pSelfFlyHndl->nFineRPY[0]);

    Serialprint("   //    Roll: ");
    Serialprint(pFineRPY[0]);
    Serialprint("   Pitch: ");
    Serialprint(pFineRPY[1]);
    Serialprint("   Yaw: ");
    Serialprint(pFineRPY[2]);
}

void _print_MagData()
{
    MagneticParam_T         *pMagParam = &(pSelfFlyHndl->nMagParam);

    Serialprint("   //   Mx:"); Serialprint(pMagParam->nRawMag[0]);
    Serialprint("   pRawMag[Y_AXIS]:"); Serialprint(pMagParam->nRawMag[1]);
    Serialprint("   Mz:"); Serialprint(pMagParam->nRawMag[2]);
    Serialprint("   Magnetic HEAD:"); Serialprint(pMagParam->nMagHeadingDeg);
    Serialprint("   SmoothHEAD:"); Serialprint(pMagParam->nSmoothHeadingDegrees);
}

void _print_BarometerData()
{
    BaroParam_T             *pBaroParam = &(pSelfFlyHndl->nBaroParam);

    Serialprint("   //    Barometer AvgTemp:"); Serialprint(pBaroParam->nAvgTemp);
    Serialprint("   AvgPress:"); Serialprint(pBaroParam->nAvgPressure);
    Serialprint("   AvgAlt:"); Serialprint(pBaroParam->nAvgAbsoluteAltitude);
    Serialprint("   RelativeAlt:"); Serialprint(pBaroParam->nRelativeAltitude);
    Serialprint("   VerticalSpeed:"); Serialprint(pBaroParam->nVerticalSpeed);
    Serialprint("   ");
}

void _print_SonarData()
{
    Serialprint("   //    Sonar: ");
    Serialprint(pSelfFlyHndl->SonicParam.nDistFromGnd);
    Serialprintln("cm   ");
}
#endif
#endif /* Debugger */

