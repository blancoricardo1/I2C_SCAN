/******************************************************************************
 * MAX32655FTHR I2C2 Address Scanner
 *
 * Scans the I2C2 bus (SCL: P0.30, SDA: P0.31) for any connected slave devices.
 * Requires external 10kΩ pull-up resistors to 1.8V on both SDA and SCL.
 ******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_delay.h"
#include "i2c.h"

#define I2C_MASTER MXC_I2C2 // Using I2C2 on FTHR
#define I2C_FREQ 100000     // 100 kHz

int main(void)
{
    printf("\n******** MAX32655FTHR I2C2 SLAVE ADDRESS SCANNER *********\n");
    printf("Scanning I2C2 on pins: SCL - P0.30, SDA - P0.31\n");
    printf("Ensure 10kΩ pull-ups to 1.8V are connected to SDA and SCL\n\n");

    if (MXC_I2C_Init(I2C_MASTER, 1, 0) != E_NO_ERROR)
    {
        printf("--> I2C2 Initialization Failed\n");
        return -1;
    }

    MXC_I2C_SetFrequency(I2C_MASTER, I2C_FREQ);
    printf("--> I2C2 Initialization Complete\n");
    printf("--> Scanning started\n");

    mxc_i2c_req_t req;
    req.i2c = I2C_MASTER;
    req.tx_buf = NULL;
    req.tx_len = 0;
    req.rx_buf = NULL;
    req.rx_len = 0;
    req.restart = 0;
    req.callback = NULL;

    int found = 0;
    for (uint8_t addr = 8; addr < 120; addr++)
    {
        req.addr = addr;
        printf(".");
        fflush(stdout);

        if (MXC_I2C_MasterTransaction(&req) == E_NO_ERROR)
        {
            printf("\nFound device at address: 0x%02X (%d)\n", addr, addr);
            found++;
        }

        MXC_Delay(MXC_DELAY_MSEC(100));
    }

    printf("\n--> Scan complete. %d device(s) found.\n", found);
    return 0;
}
