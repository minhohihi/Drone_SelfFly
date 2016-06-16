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
    Serialprint(_gDroneStatus);
}


void _print_CaturedRC_Signals()
{
    Serialprint(F("   //   RC_Roll:"));
    Serialprint(_gRCSignalVal[CH_TYPE_ROLL]);
    Serialprint(F("   RC_Pitch:"));
    Serialprint(_gRCSignalVal[CH_TYPE_PITCH]);
    Serialprint(F("   RC_Throttle:"));
    Serialprint(_gRCSignalVal[CH_TYPE_THROTTLE]);
    Serialprint(F("   RC_Yaw:"));
    Serialprint(_gRCSignalVal[CH_TYPE_YAW]);
    Serialprint(F("   RC_Gear:"));
    Serialprint(_gRCSignalVal[CH_TYPE_TAKE_LAND]);
}


void _print_CompensatedRC_Signals()
{
    Serialprint(F("   //   RC_Roll:"));
    Serialprint(_gCompensatedRCVal[CH_TYPE_ROLL]);
    Serialprint(F("   RC_Pitch:"));
    Serialprint(_gCompensatedRCVal[CH_TYPE_PITCH]);
    Serialprint(F("   RC_Throttle:"));
    Serialprint(_gCompensatedRCVal[CH_TYPE_THROTTLE]);
    Serialprint(F("   RC_Yaw:"));
    Serialprint(_gCompensatedRCVal[CH_TYPE_YAW]);
    Serialprint(F("   RC_Gear:"));
    Serialprint(_gCompensatedRCVal[CH_TYPE_TAKE_LAND]);
}


void _print_Gyro_Signals()
{
    Serialprint(F("   //   Gx:"));
    Serialprint(_gRawGyro[0]);
    Serialprint(F("   Gy:"));
    Serialprint(_gRawGyro[1]);
    Serialprint(F("   Gz:"));
    Serialprint(_gRawGyro[2]);
}


void _print_Accel_Signals()
{
    Serialprint(F("   //   Ax:"));
    Serialprint(_gRawAccel[0]);
    Serialprint(F("   Ay:"));
    Serialprint(_gRawAccel[1]);
    Serialprint(F("   Az:"));
    Serialprint(_gRawAccel[2]);
}


void _print_Temp_Signals()
{
    Serialprint(F("   //   Temp:"));
    Serialprint(_gRawTemp/340.00 + 36.53);
}


void _print_MagData()
{
    Serialprint(F("   //   Mx:"));
    Serialprint(_gRawMag[0]);
    Serialprint(F("   My:"));
    Serialprint(_gRawMag[1]);
    Serialprint(F("   Mz:"));
    Serialprint(_gRawMag[2]);
    Serialprint(F("   Magnetic HEAD:"));
    Serialprint(_gMagHeadingDeg);
    Serialprint(F("   SmoothHEAD:"));
    Serialprint(_gSmoothHeadingDegrees);
}

void _print_BarometerData()
{
    Serialprint(F("   //   Barometer AvgTemp:"));
    Serialprint(_gAvgTemp);
    Serialprint(F("   AvgPress:"));
    Serialprint(_gAvgPressure);
    Serialprint(F("   AvgAlt:"));
    Serialprint(_gAvgAbsoluteAltitude);
    Serialprint(F("   RelativeAlt:"));
    Serialprint(_gRelativeAltitude);
    Serialprint(F("   VerticalSpeed:"));
    Serialprint(_gVerticalSpeed);
}

void _print_SonarData()
{
    Serialprint(F("   //   Sonar:"));
    Serialprint(_gDistFromGnd);
    Serialprint(F("cm"));
}


void _print_RPY_Signals()
{
    Serialprint(F("   //   EstRoll:"));
    Serialprint(_gEstimatedRPY[0]);
    Serialprint(F("   EstPitch:"));
    Serialprint(_gEstimatedRPY[1]);
    Serialprint(F("   EstYaw:"));
    Serialprint(_gEstimatedRPY[2]);
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
    Serialprint(_gRPY_PID[0].nBalance);
    Serialprint(F("   PIDBal_Picth:"));
    Serialprint(_gRPY_PID[1].nBalance);
    Serialprint(F("   PIDBal_Yaw:"));
    Serialprint(_gRPY_PID[2].nBalance);
    Serialprint(F("   PIDBal_Sum:"));
    Serialprint(_gRPY_PID[0].nBalance + _gRPY_PID[1].nBalance + _gRPY_PID[2].nBalance);
}


void _print_Throttle_Signals()
{
    Serialprint(F("   //   Thrt1:"));
    Serialprint(_gESCOutput[0]);
    Serialprint(F("  Thrt2:"));
    Serialprint(_gESCOutput[1]);
    Serialprint(F("  Thrt3:"));
    Serialprint(_gESCOutput[2]);
    Serialprint(F("  Thrt4:"));
    Serialprint(_gESCOutput[3]);
}


void _print_Profile()
{
    #if __PROFILE__
    _gProfileEndTime = micros();

    Serialprint(F("   //   Loop Duration:")); 
    Serialprint((_gProfileEndTime - _gProfileStartTime) / 1000.0);

    _gProfileStartTime = micros();
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

