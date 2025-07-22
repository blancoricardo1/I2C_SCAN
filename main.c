#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_delay.h"
#include "i2c.h"

#define I2C_MASTER MXC_I2C2
#define I2C_FREQ 100000
#define LSM6DSL_ADDR 0x6B

// Registers
#define WHO_AM_I_REG 0x0F
#define CTRL1_XL 0x10
#define CTRL2_G 0x11
#define CTRL3_C 0x12
#define CTRL10_C 0x19
#define MASTER_CONFIG 0x1A
#define STATUS_REG 0x1E
#define OUTX_L_XL 0x28
#define OUTX_L_G 0x22
// Add at top (after includes)
#define GYRO_CALIB_SAMPLES 50
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

int i2c_write(uint8_t reg, uint8_t value)
{
    uint8_t buffer[2] = {reg, value};
    mxc_i2c_req_t req = {
        .i2c = I2C_MASTER,
        .addr = LSM6DSL_ADDR,
        .tx_buf = buffer,
        .tx_len = 2,
        .rx_buf = NULL,
        .rx_len = 0,
        .restart = 0,
        .callback = NULL,
    };
    int result = MXC_I2C_MasterTransaction(&req);
    printf("[DEBUG] Write reg 0x%02X = 0x%02X -> %s\n", reg, value, result == 0 ? "OK" : "FAIL");
    return result;
}

int i2c_read(uint8_t reg, uint8_t *data, int len)
{
    mxc_i2c_req_t req = {
        .i2c = I2C_MASTER,
        .addr = LSM6DSL_ADDR,
        .tx_buf = &reg,
        .tx_len = 1,
        .rx_buf = data,
        .rx_len = len,
        .restart = 1,
        .callback = NULL,
    };
    int result = MXC_I2C_MasterTransaction(&req);
    return result;
}

int main(void)
{
    printf("\n--- LSM6DSL Polling Fix Diagnostic ---\n");

    if (MXC_I2C_Init(I2C_MASTER, 1, 0) != E_NO_ERROR)
    {
        printf("I2C init failed\n");
        return -1;
    }

    MXC_I2C_SetFrequency(I2C_MASTER, I2C_FREQ);

    uint8_t whoami = 0;
    if (i2c_read(WHO_AM_I_REG, &whoami, 1) != 0 || whoami != 0x6A)
    {
        printf("WHO_AM_I: 0x%02X (Expected 0x6A)\nDevice not recognized.\n", whoami);
        return -1;
    }
    printf("WHO_AM_I: 0x%02X (Expected 0x6A)\n", whoami);

    // Soft reset
    i2c_write(CTRL3_C, 0x01);
    MXC_Delay(MXC_DELAY_MSEC(100));
    i2c_write(CTRL3_C, 0x44); // BDU=1, IF_INC=1

    // Enable accelerometer and gyro
    i2c_write(CTRL1_XL, 0x40);      // Accel ODR = 104Hz
    i2c_write(CTRL2_G, 0x48);       // Gyro ODR = 104Hz
    i2c_write(CTRL10_C, 0x38);      // Enable XL/G axes
    i2c_write(MASTER_CONFIG, 0x00); // Disable embedded functions
    printf("Sensor fully configured. Starting loop...\n");

    MXC_Delay(MXC_DELAY_MSEC(100)); // Allow sensor to settle

    printf("Calibrating gyro... keep device still.\n");
    for (int i = 0; i < GYRO_CALIB_SAMPLES; i++)
    {
        uint8_t raw[6];
        for (int j = 0; j < 6; ++j)
            i2c_read(OUTX_L_G + j, &raw[j], 1);
        int16_t gx = (int16_t)(raw[1] << 8 | raw[0]);
        int16_t gy = (int16_t)(raw[3] << 8 | raw[2]);
        int16_t gz = (int16_t)(raw[5] << 8 | raw[4]);

        gx_offset += gx * 8.75f / 1000.0f;
        gy_offset += gy * 8.75f / 1000.0f;
        gz_offset += gz * 8.75f / 1000.0f;

        MXC_Delay(MXC_DELAY_MSEC(50));
    }
    gx_offset /= GYRO_CALIB_SAMPLES;
    gy_offset /= GYRO_CALIB_SAMPLES;
    gz_offset /= GYRO_CALIB_SAMPLES;
    printf("Gyro bias: X=%.2f, Y=%.2f, Z=%.2f\n", gx_offset, gy_offset, gz_offset);

    while (1)
    {
        MXC_Delay(MXC_DELAY_MSEC(10));

        uint8_t status = 0;
        i2c_read(STATUS_REG, &status, 1);

        if (!(status & 0x03))
        {
            MXC_Delay(MXC_DELAY_MSEC(10));
            continue;
        }

        // Read accel
        uint8_t acc_raw[6];
        for (int i = 0; i < 6; ++i)
            i2c_read(OUTX_L_XL + i, &acc_raw[i], 1);
        int16_t ax = (int16_t)(acc_raw[1] << 8 | acc_raw[0]);
        int16_t ay = (int16_t)(acc_raw[3] << 8 | acc_raw[2]);
        int16_t az = (int16_t)(acc_raw[5] << 8 | acc_raw[4]);

        // Read gyro
        uint8_t gyr_raw[6];
        for (int i = 0; i < 6; ++i)
            i2c_read(OUTX_L_G + i, &gyr_raw[i], 1);
        int16_t gx = (int16_t)(gyr_raw[1] << 8 | gyr_raw[0]);
        int16_t gy = (int16_t)(gyr_raw[3] << 8 | gyr_raw[2]);
        int16_t gz = (int16_t)(gyr_raw[5] << 8 | gyr_raw[4]);

        // Convert accel
        float ax_g = ax * 0.061f / 1000.0f;
        float ay_g = ay * 0.061f / 1000.0f;
        float az_g = az * 0.061f / 1000.0f;

        // Convert gyro (raw) and subtract bias
        float gx_dps = gx * 8.75f / 1000.0f - gx_offset;
        float gy_dps = gy * 8.75f / 1000.0f - gy_offset;
        float gz_dps = gz * 8.75f / 1000.0f - gz_offset;

        printf("Accel X: %.3f, Y: %.3f, Z: %.3f | Gyro X: %.2f, Y: %.2f, Z: %.2f\n",
               ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps);

        fflush(stdout);
        MXC_Delay(MXC_DELAY_MSEC(200));
    }
    return 0;
}
