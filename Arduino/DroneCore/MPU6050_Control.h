//
//  MPU6050_Controller.h
//  SelfFly
//
//  Created by Maverick on 2016. 1. 21..
//

#ifndef __MPU6050_CONTROL__
#define __MPU6050_CONTROL__


void _AccelGyro_Initialize()
{
    pSelfFlyHndl->nAccelGyroHndl = MPU6050();

    Serialprintln(F(" Initializing MPU..."));
    pSelfFlyHndl->nAccelGyroHndl.initialize();

    // Verify Vonnection
    Serialprint(F("    Testing device connections..."));
    Serialprintln(pSelfFlyHndl->nAccelGyroHndl.testConnection() ? F("  MPU6050 connection successful") : F("  MPU6050 connection failed"));

    pSelfFlyHndl->nAccelGyroHndl.setI2CMasterModeEnabled(false);
    pSelfFlyHndl->nAccelGyroHndl.setI2CBypassEnabled(true);
    pSelfFlyHndl->nAccelGyroHndl.setSleepEnabled(false);

    // Calibrate GyroAccel
    //pSelfFlyHndl->nAccelGyroHndl.doCalibration();
    pSelfFlyHndl->nAccelGyroHndl.setXGyroOffset(MPU6050_GYRO_OFFSET_X);
    pSelfFlyHndl->nAccelGyroHndl.setYGyroOffset(MPU6050_GYRO_OFFSET_Y);
    pSelfFlyHndl->nAccelGyroHndl.setZGyroOffset(MPU6050_GYRO_OFFSET_Z);
    pSelfFlyHndl->nAccelGyroHndl.setXAccelOffset(MPU6050_ACCEL_OFFSET_X);
    pSelfFlyHndl->nAccelGyroHndl.setYAccelOffset(MPU6050_ACCEL_OFFSET_Y);
    //pSelfFlyHndl->nAccelGyroHndl.setZAccelOffset(MPU6050_ACCEL_OFFSET_Z);

    // supply your own gyro offsets here, scaled for min sensitivity
    pSelfFlyHndl->nAccelGyroHndl.setRate(1);                                            // Sample Rate (500Hz = 1Hz Gyro SR / 1+1)
    pSelfFlyHndl->nAccelGyroHndl.setDLPFMode(MPU6050_DLPF_BW_20);                       // Low Pass filter 20hz
    pSelfFlyHndl->nAccelGyroHndl.setFullScaleGyroRange(GYRO_FS_PRECISIOM);              // 250? / s (MPU6050_GYRO_FS_250)
    pSelfFlyHndl->nAccelGyroHndl.setFullScaleAccelRange(ACCEL_FS_PRECISIOM);            // +-2g (MPU6050_ACCEL_FS_2)

    Serialprintln(F(" MPU Initialized!!!"));

    return;
}


void _AccelGyro_GetData()
{
    AccelGyroParam_T        *pAccelGyroParam = &(pSelfFlyHndl->nAccelGyroParam);
    float                   *pRawGyro = &(pAccelGyroParam->nRawGyro[X_AXIS]);
    float                   *pRawAccel = &(pAccelGyroParam->nRawAccel[X_AXIS]);

    // Read Gyro and Accelerate Data
    Wire.beginTransmission(MPU6050_ADDRESS_AD0_LOW);
    Wire.write(MPU6050_RA_ACCEL_XOUT_H);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDRESS_AD0_LOW, 14, true);  // request a total of 14 registers

    pRawAccel[X_AXIS] = (float)(Wire.read()<<8 | Wire.read());               // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    pRawAccel[Y_AXIS] = (float)(Wire.read()<<8 | Wire.read());               // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    pRawAccel[Z_AXIS] = (float)(Wire.read()<<8 | Wire.read());               // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    pAccelGyroParam->nRawTemp = (float)(Wire.read()<<8 | Wire.read());       // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)        340 per degrees Celsius, -512 at 35 degrees
    pRawGyro[X_AXIS]= (float)(Wire.read()<<8 | Wire.read());                 // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    pRawGyro[Y_AXIS]= (float)(Wire.read()<<8 | Wire.read());                 // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    pRawGyro[Z_AXIS]= (float)(Wire.read()<<8 | Wire.read());                 // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

#endif /* MPU6050_Controller_h */
