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
    // printf("[DEBUG] Read reg 0x%02X (%d bytes) -> %s\n", reg, len, result == 0 ? "OK" : "FAIL");
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
    i2c_write(CTRL1_XL, 0x40); // Accel ODR = 104Hz
    i2c_write(CTRL2_G, 0x40);  // Gyro ODR = 104Hz
    i2c_write(CTRL10_C, 0x38); // Enable XL/G axes

    i2c_write(MASTER_CONFIG, 0x00); // Disable embedded functions
    printf("Sensor fully configured. Starting loop...\n");

    MXC_Delay(MXC_DELAY_MSEC(100)); // Allow sensor to settle

    while (1)
    {
        MXC_Delay(MXC_DELAY_MSEC(10)); // Add delay to avoid flooding I2C

        uint8_t status = 0;
        i2c_read(STATUS_REG, &status, 1);

        if (!(status & 0x01))
        {
            MXC_Delay(MXC_DELAY_MSEC(10));
            continue;
        }

        uint8_t raw[6];
        for (int i = 0; i < 6; ++i)
        {
            i2c_read(OUTX_L_XL + i, &raw[i], 1);
        }

        int16_t ax = (int16_t)(raw[1] << 8 | raw[0]);
        int16_t ay = (int16_t)(raw[3] << 8 | raw[2]);
        int16_t az = (int16_t)(raw[5] << 8 | raw[4]);

        printf("Accel X: %d, Y: %d, Z: %d\n", ax, ay, az);
        fflush(stdout);
        MXC_Delay(MXC_DELAY_MSEC(200));
    }

    return 0;
}