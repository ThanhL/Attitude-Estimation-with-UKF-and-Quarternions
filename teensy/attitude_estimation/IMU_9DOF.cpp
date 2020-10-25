/***  
LSM6DS33 LIS3MDL 9DOF IMU Wrapper

This is a wrapper for IMU models LSM6DS33 and LIS3MDL. This wrapper is intended to keep code
clean and concise when initializing the separate sensors (LSM6DS33 and LIS3MDL) into one place
***/
#include "Arduino.h"
#include "IMU_9DOF.h"

IMU_9DOF::IMU_9DOF()
{
    // Constructor
}

IMU_9DOF::~IMU_9DOF()
{
    // Destructor
}


void IMU_9DOF::init_sensors_i2c(void)
{
    /***
    Initializes the LSM6DS33 & LIS3MDL over I2C.
    ***/
    while (!lsm6ds.begin_I2C(LSM6DS33_I2C_ADDR))
    {
        Serial.println("Failed to find LSM6DS chip. Retrying...");
        delay(1000);
    }

    while (!lis3mdl.begin_I2C(LIS3MDL_I2C_ADDR))
    {
        Serial.println("Failed to find LIS3MDL chip. Retrying...");
        delay(1000);   
    }
    Serial.println("LSM6DS and LIS3MDL Found!");    
}

void IMU_9DOF::get_accelerometer_settings(void)
{
    /***
    Outputs the current accelerometer settings
    ***/
    Serial.print("Accelerometer range set to: ");
    switch (lsm6ds.getAccelRange()) 
    {
        case LSM6DS_ACCEL_RANGE_2_G:
            Serial.println("+-2G");
            break;
        case LSM6DS_ACCEL_RANGE_4_G:
            Serial.println("+-4G");
            break;
        case LSM6DS_ACCEL_RANGE_8_G:
            Serial.println("+-8G");
            break;
        case LSM6DS_ACCEL_RANGE_16_G:
            Serial.println("+-16G");
            break;
    }

    Serial.print("Accelerometer data rate set to: ");
    switch (lsm6ds.getAccelDataRate()) 
    {
        case LSM6DS_RATE_SHUTDOWN:
            Serial.println("0 Hz");
            break;
        case LSM6DS_RATE_12_5_HZ:
            Serial.println("12.5 Hz");
            break;
        case LSM6DS_RATE_26_HZ:
            Serial.println("26 Hz");
            break;
        case LSM6DS_RATE_52_HZ:
            Serial.println("52 Hz");
            break;
        case LSM6DS_RATE_104_HZ:
            Serial.println("104 Hz");
            break;
        case LSM6DS_RATE_208_HZ:
            Serial.println("208 Hz");
            break;
        case LSM6DS_RATE_416_HZ:
            Serial.println("416 Hz");
            break;
        case LSM6DS_RATE_833_HZ:
            Serial.println("833 Hz");
            break;
        case LSM6DS_RATE_1_66K_HZ:
            Serial.println("1.66 KHz");
            break;
        case LSM6DS_RATE_3_33K_HZ:
            Serial.println("3.33 KHz");
            break;
        case LSM6DS_RATE_6_66K_HZ:
            Serial.println("6.66 KHz");
            break;
    }
}


void IMU_9DOF::get_gyroscope_settings(void)
{
    /***
    Outputs the current gyroscope settings
    ***/    
    Serial.print("Gyro range set to: ");
    switch (lsm6ds.getGyroRange()) 
    {
        case LSM6DS_GYRO_RANGE_125_DPS:
            Serial.println("125 degrees/s");
            break;
        case LSM6DS_GYRO_RANGE_250_DPS:
            Serial.println("250 degrees/s");
            break;
        case LSM6DS_GYRO_RANGE_500_DPS:
            Serial.println("500 degrees/s");
            break;
        case LSM6DS_GYRO_RANGE_1000_DPS:
            Serial.println("1000 degrees/s");
            break;
        case LSM6DS_GYRO_RANGE_2000_DPS:
            Serial.println("2000 degrees/s");
            break;
        case ISM330DHCX_GYRO_RANGE_4000_DPS:
            Serial.println("4000 degrees/s");
            break;
    }
      
    Serial.print("Gyro data rate set to: ");
    switch (lsm6ds.getGyroDataRate()) 
    {
        case LSM6DS_RATE_SHUTDOWN:
            Serial.println("0 Hz");
            break;
        case LSM6DS_RATE_12_5_HZ:
            Serial.println("12.5 Hz");
            break;
        case LSM6DS_RATE_26_HZ:
            Serial.println("26 Hz");
            break;
        case LSM6DS_RATE_52_HZ:
            Serial.println("52 Hz");
            break;
        case LSM6DS_RATE_104_HZ:
            Serial.println("104 Hz");
            break;
        case LSM6DS_RATE_208_HZ:
            Serial.println("208 Hz");
            break;
        case LSM6DS_RATE_416_HZ:
            Serial.println("416 Hz");
            break;
        case LSM6DS_RATE_833_HZ:
            Serial.println("833 Hz");
            break;
        case LSM6DS_RATE_1_66K_HZ:
            Serial.println("1.66 KHz");
            break;
        case LSM6DS_RATE_3_33K_HZ:
            Serial.println("3.33 KHz");
            break;
        case LSM6DS_RATE_6_66K_HZ:
            Serial.println("6.66 KHz");
            break;
    }
}

void IMU_9DOF::get_magnetometer_settings(void)
{
    /***
    Outputs the current magnetometer settings
    ***/    
    Serial.print("Magnetometer data rate set to: ");
    switch (lis3mdl.getDataRate()) 
    {
        case LIS3MDL_DATARATE_0_625_HZ: 
            Serial.println("0.625 Hz"); 
            break;
        case LIS3MDL_DATARATE_1_25_HZ: 
            Serial.println("1.25 Hz"); 
            break;
        case LIS3MDL_DATARATE_2_5_HZ: 
            Serial.println("2.5 Hz"); 
            break;
        case LIS3MDL_DATARATE_5_HZ: 
            Serial.println("5 Hz"); 
            break;
        case LIS3MDL_DATARATE_10_HZ: 
            Serial.println("10 Hz"); 
            break;
        case LIS3MDL_DATARATE_20_HZ: 
            Serial.println("20 Hz"); 
            break;
        case LIS3MDL_DATARATE_40_HZ: 
            Serial.println("40 Hz"); 
            break;
        case LIS3MDL_DATARATE_80_HZ: 
            Serial.println("80 Hz"); 
            break;
        case LIS3MDL_DATARATE_155_HZ: 
            Serial.println("155 Hz"); 
            break;
        case LIS3MDL_DATARATE_300_HZ: 
            Serial.println("300 Hz"); 
            break;
        case LIS3MDL_DATARATE_560_HZ: 
            Serial.println("560 Hz"); 
            break;
        case LIS3MDL_DATARATE_1000_HZ: 
            Serial.println("1000 Hz"); 
            break;
    }

    Serial.print("Magnetometer Range set to: ");
    switch (lis3mdl.getRange()) 
    {
        case LIS3MDL_RANGE_4_GAUSS: 
            Serial.println("+-4 gauss"); 
            break;
        case LIS3MDL_RANGE_8_GAUSS: 
            Serial.println("+-8 gauss"); 
            break;
        case LIS3MDL_RANGE_12_GAUSS: 
            Serial.println("+-12 gauss"); 
            break;
        case LIS3MDL_RANGE_16_GAUSS: 
            Serial.println("+-16 gauss"); 
            break;
    }

    Serial.print("Magnetometer performance mode set to: ");
    switch (lis3mdl.getPerformanceMode()) 
    {
        case LIS3MDL_LOWPOWERMODE: 
            Serial.println("Low"); 
            break;
        case LIS3MDL_MEDIUMMODE: 
            Serial.println("Medium"); 
            break;
        case LIS3MDL_HIGHMODE: 
            Serial.println("High"); 
            break;
        case LIS3MDL_ULTRAHIGHMODE: 
            Serial.println("Ultra-High"); 
            break;
    }


    Serial.print("Magnetometer operation mode set to: ");
    // Single shot mode will complete conversion and go into power down
    switch (lis3mdl.getOperationMode()) 
    {
        case LIS3MDL_CONTINUOUSMODE: 
            Serial.println("Continuous"); 
            break;
        case LIS3MDL_SINGLEMODE: 
            Serial.println("Single mode"); 
            break;
        case LIS3MDL_POWERDOWNMODE: 
            Serial.println("Power-down"); 
            break;
    }
}



