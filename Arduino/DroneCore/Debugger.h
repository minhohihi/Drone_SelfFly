//
//  Debugger.h
//  SelfFly
//
//  Created by Maverick on 2016. 1. 21..
//

#ifndef __DEBUGGER__
#define __DEBUGGER__

#if __PRINT_DEBUG__ || __EXTERNAL_READ__
void _print_EEPRomData()
{
    int         i = 0;
    
    
}


void _print_DroneStatus()
{
    Serialprint(F("   //   Drone Status:")); 
    Serialprint(nDroneStatus);
}


void _print_CaturedRC_Signals()
{
    Serialprint(F("   //   RC_Roll:"));
    Serialprint(nRcvChVal[CH_TYPE_ROLL]);
    Serialprint(F("   RC_Pitch:"));
    Serialprint(nRcvChVal[CH_TYPE_PITCH]);
    Serialprint(F("   RC_Throttle:"));
    Serialprint(nRcvChVal[CH_TYPE_THROTTLE]);
    Serialprint(F("   RC_Yaw:"));
    Serialprint(nRcvChVal[CH_TYPE_YAW]);
    Serialprint(F("   RC_Gear:"));
    Serialprint(nRcvChVal[CH_TYPE_TAKE_LAND]);
}


void _print_CompensatedRC_Signals()
{
    Serialprint(F("   //   RC_Roll:"));
    Serialprint(nCompensatedRCVal[CH_TYPE_ROLL]);
    Serialprint(F("   RC_Pitch:"));
    Serialprint(nCompensatedRCVal[CH_TYPE_PITCH]);
    Serialprint(F("   RC_Throttle:"));
    Serialprint(nCompensatedRCVal[CH_TYPE_THROTTLE]);
    Serialprint(F("   RC_Yaw:"));
    Serialprint(nCompensatedRCVal[CH_TYPE_YAW]);
    Serialprint(F("   RC_Gear:"));
    Serialprint(nCompensatedRCVal[CH_TYPE_TAKE_LAND]);
}


void _print_Gyro_Signals()
{
    Serialprint(F("   //   Gx:"));
    Serialprint(nRawGyro[0]);
    Serialprint(F("   Gy:"));
    Serialprint(nRawGyro[1]);
    Serialprint(F("   Gz:"));
    Serialprint(nRawGyro[2]);
}


void _print_Accel_Signals()
{
    Serialprint(F("   //   Ax:"));
    Serialprint(nRawAccel[0]);
    Serialprint(F("   Ay:"));
    Serialprint(nRawAccel[1]);
    Serialprint(F("   Az:"));
    Serialprint(nRawAccel[2]);
}


void _print_Temp_Signals()
{
    Serialprint(F("   //   Temp:"));
    Serialprint(nRawTemp/340.00 + 36.53);
}


void _print_MagData()
{
    Serialprint(F("   //   Mx:"));
    Serialprint(nRawMag[0]);
    Serialprint(F("   My:"));
    Serialprint(nRawMag[1]);
    Serialprint(F("   Mz:"));
    Serialprint(nRawMag[2]);
    Serialprint(F("   Magnetic HEAD:"));
    Serialprint(nMagHeadingDeg);
    Serialprint(F("   SmoothHEAD:"));
    Serialprint(nSmoothHeadingDegrees);
}

void _print_BarometerData()
{
    Serialprint(F("   //   Barometer AvgTemp:"));
    Serialprint(nAvgTemp);
    Serialprint(F("   AvgPress:"));
    Serialprint(nAvgPressure);
    Serialprint(F("   AvgAlt:"));
    Serialprint(nAvgAbsoluteAltitude);
    Serialprint(F("   RelativeAlt:"));
    Serialprint(nRelativeAltitude);
    Serialprint(F("   VerticalSpeed:"));
    Serialprint(nVerticalSpeed);
}

void _print_SonarData()
{
    Serialprint(F("   //   Sonar:"));
    Serialprint(nDistFromGnd);
    Serialprint(F("cm"));
}


void _print_RPY_Signals()
{
    Serialprint(F("   //   EstRoll:"));
    Serialprint(nEstimatedRPY[0]);
    Serialprint(F("   EstPitch:"));
    Serialprint(nEstimatedRPY[1]);
    Serialprint(F("   EstYaw:"));
    Serialprint(nEstimatedRPY[2]);
}


void _print_PIDGain()
{
    Serialprint(F("   //   PIDGain"));
    Serialprint(F("   P:"));
    Serialprint(nPIDGainTable[0][0]);
    Serialprint(F("   I:"));
    Serialprint(nPIDGainTable[0][1]);
    Serialprint(F("   D:"));
    Serialprint(nPIDGainTable[0][2]);
    Serialprint(F("   Rest:"));
    Serialprint(nPIDGainTable[0][3]);
}

void _print_PIDBalance()
{
    Serialprint(F("   //   PIDBal_Roll:"));
    Serialprint(nRPY_PID[0].nBalance);
    Serialprint(F("   PIDBal_Picth:"));
    Serialprint(nRPY_PID[1].nBalance);
    Serialprint(F("   PIDBal_Yaw:"));
    Serialprint(nRPY_PID[2].nBalance);
    Serialprint(F("   PIDBal_Sum:"));
    Serialprint(nRPY_PID[0].nBalance + nRPY_PID[1].nBalance + nRPY_PID[2].nBalance);
}


void _print_Throttle_Signals()
{
    Serialprint(F("   //   Thrt1:"));
    Serialprint(nESCOutput[0]);
    Serialprint(F("  Thrt2:"));
    Serialprint(nESCOutput[1]);
    Serialprint(F("  Thrt3:"));
    Serialprint(nESCOutput[2]);
    Serialprint(F("  Thrt4:"));
    Serialprint(nESCOutput[3]);
}


void _print_Profile()
{
    #if __PROFILE__
    nProfileEndTime = micros();

    Serialprint(F("   //   Loop Duration:")); 
    Serialprint((nProfileEndTime - nProfileStartTime) / 1000.0);

    nProfileStartTime = micros();
    #endif
}

void _print_Data()
{
    //_print_DroneStatus();
    //_print_CaturedRC_Signals();
    //_print_CompensatedRC_Signals();
    //_print_Gyro_Signals();
    //_print_Accel_Signals();
    //_print_Temp_Signals();
    //_print_MagData();
    //_print_BarometerData();
    //_print_SonarData();
    //_print_RPY_Signals();
    _print_PIDGain();
    //_print_PIDBalance();
    //_print_Throttle_Signals();
    //_print_Profile();
    Serialprintln(F("  "));
}
#endif
#endif /* Debugger */

