#include "imu/imu.h"
#include "imu/mpu6050.h"
#include "soft_i2c.h"
#include "main.h"
#include <math.h>

// ----------------------------------------------------------------
// Constantes et définitions
// ----------------------------------------------------------------
#define MPU6050_ADDRESS        0x68
#define MPU6050_SMPRT_DIV      0x19
#define MPU6050_CONFIG         0x1a
#define MPU6050_ACCEL_XOUT_H   0x3b
#define MPU6050_GYRO_XOUT_H    0x43
#define MPU6050_PWR_MGMT_1     0x6b
#define MPU6050_WHO_AM_I       0x75
#define MPU6500_WHO_AM_I           0x70
#define MPU9250_WHO_AM_I           0x71
#define MPU9255_WHO_AM_I           0x73

#define MPU6050_DPS_FACTOR   (1.0f / 131.0f)
#define MPU6050_G_FACTOR     (1.0f / 16384.0f)

#ifndef MS2_PER_G
#define MS2_PER_G 9.80665f
#endif

#ifndef RAD_PER_G
#define RAD_PER_G (3.14159265f / 180.0f)
#endif

#ifndef DISABLE_MPU6050


// ----------------------------------------------------------------
// Fonctions utilitaires
// ----------------------------------------------------------------

// ----------------------------------------------------------------
// Détection du capteur
// ----------------------------------------------------------------
/**
  * @brief  Test Device 
  * Perform any tests possible before actually enabling and using the device,
  * for example check the i2c address and whoami registers if existing  
  *
  * @retval          0 -> test failed 1-> test ok, good to init and use
  */
uint8_t MPU6050_TestDevice(void)
{
    uint8_t val = SW_I2C_UTIL_Read(MPU6050_ADDRESS, MPU6050_WHO_AM_I);
    /* Test who am I */
    if (val == MPU6050_WHO_AM_I || val == MPU6500_WHO_AM_I ||
        val == MPU9250_WHO_AM_I || val == MPU9255_WHO_AM_I) {
        return 1;
    }

    debug_printf("    > [MPU-6050] - Error probing for (Gyro / Accelerometer) at I2C addr=0x%02x (WHO_AM_I = 0x%02x)\r\n", MPU6050_ADDRESS, val);
    return 0;
}

// ----------------------------------------------------------------
// Initialisation
// ----------------------------------------------------------------
void MPU6050_Init(void)
{
    SW_I2C_UTIL_WRITE(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 0b00001001);  // disable temp, use gyro clock
    SW_I2C_UTIL_WRITE(MPU6050_ADDRESS, MPU6050_CONFIG, 0x05);            // LPF = 10Hz
    SW_I2C_UTIL_WRITE(MPU6050_ADDRESS, MPU6050_SMPRT_DIV, 9);            // 100 Hz

#ifdef DEBUG
    debug_printf(" * MPU6050 initialized at I2C addr 0x%02X\r\n", MPU6050_ADDRESS);
#endif
}

// ----------------------------------------------------------------
// Lecture Accéléromètre (en m/s²)
// ----------------------------------------------------------------
/**
  * @brief  Reads the 3 accelerometer channels and stores them in *x,*y,*z
  * units are m/s^2
  */
void MPU6050_ReadAccelerometerRaw(float *x, float *y, float *z)
{
    uint8_t accel_xyz[6];// 2 bytes each
    SW_I2C_UTIL_Read_Multi(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, 6, accel_xyz);

    *x =  (int16_t)(accel_xyz[0] << 8 | accel_xyz[1]) * MPU6050_G_FACTOR * MS2_PER_G;
    *y =  (int16_t)(accel_xyz[2] << 8 | accel_xyz[3]) * MPU6050_G_FACTOR * MS2_PER_G;
    *z =  (int16_t)(accel_xyz[4] << 8 | accel_xyz[5]) * MPU6050_G_FACTOR * MS2_PER_G;
}
/**
  * @brief  Reads the 3 gyro channels and stores them in *x,*y,*z
  * units are rad/sec
  */
// ----------------------------------------------------------------
// Lecture Gyroscope (en rad/s)
// ----------------------------------------------------------------
void MPU6050_ReadGyroRaw(float *x, float *y, float *z)
{
    uint8_t gyro_xyz[6];// 2 bytes each
    SW_I2C_UTIL_Read_Multi(MPU6050_ADDRESS, MPU6050_GYRO_XOUT_H, 6, gyro_xyz);

    *x = (int16_t)(gyro_xyz[0] << 8 | gyro_xyz[1]) * MPU6050_DPS_FACTOR * RAD_PER_G;
    *y = (int16_t)(gyro_xyz[2] << 8 | gyro_xyz[3]) * MPU6050_DPS_FACTOR * RAD_PER_G;
    *z = (int16_t)(gyro_xyz[4] << 8 | gyro_xyz[5]) * MPU6050_DPS_FACTOR * RAD_PER_G;
}

#endif