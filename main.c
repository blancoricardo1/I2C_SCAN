#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "mxc_device.h"
#include "mxc_delay.h"
#include "i2c.h"
#include "gpio.h"
#include "tmr.h"

#include "MAX30009.h"
#include "MAX32655.h"
#include "bioZ.h"
#include "board.h"
#include "cli.h"
#include "dma.h"
#include "gpio.h"
#include "led.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "mxc_pins.h"
#include "nvic_table.h"
#include "sdhc.h"
#include "spi.h"
#include "spiFunctions.h"
#include "rtc.h"
#include "tmr.h"
#include "uart.h"
#include "user-cli.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "wsf_types.h"
#include "wsf_trace.h"
#include "wsf_bufio.h"
#include "wsf_msg.h"
#include "wsf_assert.h"
#include "wsf_buf.h"
#include "wsf_heap.h"
#include "wsf_cs.h"
#include "wsf_timer.h"
#include "wsf_os.h"

#include "sec_api.h"
#include "hci_handler.h"
#include "dm_handler.h"
#include "l2c_handler.h"
#include "att_handler.h"
#include "smp_handler.h"
#include "l2c_api.h"
#include "att_api.h"
#include "smp_api.h"
#include "app_api.h"
#include "hci_core.h"
#include "app_terminal.h"
#include "wut.h"
#include "rtc.h"
#include "trimsir_regs.h"

#if defined(HCI_TR_EXACTLE) && (HCI_TR_EXACTLE == 1)
#include "ll_init_api.h"
#endif

#include "pal_bb.h"
#include "pal_cfg.h"
#include "app_ui.h"
/*** WIRING / ELECTRICAL TOGGLES ***/
#define INT_ACTIVE_LOW   0   // 0: active-high (LSM6DSL default), 1: active-low
#define INT_OPEN_DRAIN   0   // 0: push-pull (default), 1: open-drain (add pull-up)
#define USE_VDDIOH_I2C2  0   // 1 if your I2C bus is 3.3V and you need VDDIOH on P0.30/31

/*** I2C / LSM6DSL ***/
#define I2C_MASTER     MXC_I2C2
#define I2C_FREQ       100000
#define LSM6DSL_ADDR   0x6B           // use 0x6A if SA0/SDO is low

// Registers
#define WHO_AM_I_REG       0x0F
#define CTRL1_XL           0x10
#define CTRL2_G            0x11
#define CTRL3_C            0x12
#define INT1_CTRL          0x0D
#define DRDY_PULSE_CFG_G   0x0B
#define STATUS_REG         0x1E
#define OUTX_L_XL          0x28

/*** INT1 pin: P1_8 (GPIO1.8) ***/
#define IMU_INT_PORT   MXC_GPIO1
#define IMU_INT_PIN    MXC_GPIO_PIN_8     // mask bit for pin 8
#define IMU_INT_MASK   IMU_INT_PIN

static volatile int imu_drdy_flag = 0;
bool current_freq = 0;
volatile bool recording = false;
uint8_t gReadBuf[100];
uint8_t gHold[100];
int errCnt;
bool interrupt = 0;
extern uint32_t sample_interval_us;
extern sample_index;
int samples_discarded;
static wsfBufPoolDesc_t mainPoolDesc[] = {{16, 8}, {32, 4}, {192, 8}, {256, 16}};
bool sample_ready = 0;
#if defined(HCI_TR_EXACTLE) && (HCI_TR_EXACTLE == 1)
static LlRtCfg_t mainLlRtCfg;
#endif
volatile int wutTrimComplete;

static int i2c_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    mxc_i2c_req_t req = {
        .i2c=I2C_MASTER, .addr=LSM6DSL_ADDR,
        .tx_buf=buf, .tx_len=2, .rx_buf=NULL, .rx_len=0,
        .restart=0, .callback=NULL
    };
    int r = MXC_I2C_MasterTransaction(&req);
    printf("[DEBUG] Write reg 0x%02X = 0x%02X -> %s\n", reg, val, r==0?"OK":"FAIL");
    return r;
}
static int i2c_read(uint8_t reg, uint8_t *data, int len) {
    mxc_i2c_req_t req = {
        .i2c=I2C_MASTER, .addr=LSM6DSL_ADDR,
        .tx_buf=&reg, .tx_len=1, .rx_buf=data, .rx_len=len,
        .restart=1, .callback=NULL
    };
    return MXC_I2C_MasterTransaction(&req);
}

/*** GPIO ISR ***/
static void imuISR(void *unused)
{
    MXC_GPIO_ClearFlags(IMU_INT_PORT, IMU_INT_MASK);
    MXC_GPIO_DisableInt(IMU_INT_PORT, IMU_INT_MASK);   // <--- stop level-held retriggers
    imu_drdy_flag = 1;
}
void GPIO1_IRQHandler(void) { MXC_GPIO_Handler(1); }

static void setup_gpio_int_config_only(void)
{
    mxc_gpio_cfg_t pin = {
        .port  = IMU_INT_PORT,
        .mask  = IMU_INT_MASK,
        .func  = MXC_GPIO_FUNC_IN,
        .pad   = MXC_GPIO_PAD_NONE,
        .vssel = MXC_GPIO_VSSEL_VDDIO
    };
    MXC_GPIO_Config(&pin);
    MXC_GPIO_ClearFlags(pin.port, pin.mask);
    MXC_GPIO_RegisterCallback(&pin, imuISR, NULL);

    // Latched DRDY => use LEVEL-HIGH (active-low? use MXC_GPIO_INT_LOW and set H_LACTIVE)
    MXC_GPIO_IntConfig(&pin, MXC_GPIO_INT_HIGH);
    // DO NOT: MXC_GPIO_EnableInt(...) yet
    // DO NOT: NVIC_EnableIRQ(...) yet
}

// --- After sensor config is done, THEN enable GPIO + NVIC ---
static void enable_imu_gpio_irq(void)
{
    MXC_GPIO_ClearFlags(IMU_INT_PORT, IMU_INT_MASK);
    MXC_GPIO_EnableInt(IMU_INT_PORT, IMU_INT_MASK);
    NVIC_ClearPendingIRQ(MXC_GPIO_GET_IRQ(1));
    NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(1));
}
void buttonISR(void *unused)
{
  // 1. Clear the GPIO interrupt flag
  MXC_GPIO_ClearFlags(MXC_GPIO0, MXC_GPIO_PIN_2);

  if (!recording)
  {
    // datsSendData(AppConnIsOpen(), "startPhys\n", sizeof("startPhys\n") - 1);
    recording = 1;
  }
  else
  {
    // datsSendData(AppConnIsOpen(), "stopPhys\n", sizeof("stopPhys\n") - 1);
    recording = 0;
  }
}
void setupButtonInterrupt()
{
  mxc_gpio_cfg_t intaPin = {
      .port = MXC_GPIO0,
      .mask = MXC_GPIO_PIN_2,
      .func = MXC_GPIO_FUNC_IN,
      .pad = MXC_GPIO_PAD_PULL_UP,  // <-- Enable internal pull-up
      .vssel = MXC_GPIO_VSSEL_VDDIO // or VDDIOH if you use 3.3V IO
  };

  MXC_GPIO_Config(&intaPin);

  MXC_GPIO_RegisterCallback(&intaPin, buttonISR, NULL);
  MXC_GPIO_IntConfig(&intaPin, MXC_GPIO_INT_FALLING); // Detect falling edge
  MXC_GPIO_EnableInt(intaPin.port, intaPin.mask);

  NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(0)); // Enable NVIC interrupt for GPIO0 block
}
void sensorISR(void *unused)
{

  MXC_GPIO_ClearFlags(MXC_GPIO0, MXC_GPIO_PIN_25); // Clear interrupt

  regRead(0x00);
  // uint32_t t_start = MXC_TMR_GetCount(MXC_TMR1);
  interrupt = 1; // Set interrupt flag to indicate that a sample was discarded

  if (samples_discarded == 7)
  {
    sample_ready = 1;
    samples_discarded = 0;
  }
}

void setupMax30009Interrupt(void)
{
  mxc_gpio_cfg_t intbPin = {
      .port = MXC_GPIO0,
      .mask = MXC_GPIO_PIN_25,
      .func = MXC_GPIO_FUNC_IN,
      .pad = MXC_GPIO_PAD_PULL_UP,  // <-- Enable internal pull-up
      .vssel = MXC_GPIO_VSSEL_VDDIO // or VDDIOH if you use 3.3V IO
  };

  MXC_GPIO_Config(&intbPin);

  MXC_GPIO_RegisterCallback(&intbPin, sensorISR, NULL);
  MXC_GPIO_IntConfig(&intbPin, MXC_GPIO_INT_FALLING); // Detect falling edge
  MXC_GPIO_EnableInt(intbPin.port, intbPin.mask);

  NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(0)); // Enable NVIC interrupt for GPIO0 block
}

/*** MAIN ***/
int main(void)
{
    printf("\n--- LSM6DSL (latched DRDY, level-high) ---\n");

MXC_I2C_Init(I2C_MASTER, 1, 0);
MXC_I2C_SetFrequency(I2C_MASTER, I2C_FREQ);

// Probe
uint8_t who=0;
i2c_read(WHO_AM_I_REG, &who, 1);
printf("WHO_AM_I: 0x%02X\n", who);

// 1) Prepare GPIO (but do NOT enable yet)
setup_gpio_int_config_only();

// 2) Sensor reset + interface
i2c_write(CTRL3_C, 0x01);
MXC_Delay(MXC_DELAY_MSEC(100));
i2c_write(CTRL3_C, 0x44);   // BDU=1, IF_INC=1; active-high push-pull

// 3) Accel on, gyro off
i2c_write(CTRL2_G, 0x00);
i2c_write(CTRL1_XL, 0x40);  // 104 Hz

// 4) Explicit latched DRDY (pulse off)
i2c_write(DRDY_PULSE_CFG_G, 0x00);

// 5) Route XL-DRDY -> INT1
i2c_write(INT1_CTRL, 0x01);

// Optional: one dummy read clears any stale latch before enabling IRQ
uint8_t st=0; i2c_read(STATUS_REG, &st, 1);


// 6) NOW enable GPIO+NVIC
enable_imu_gpio_irq();
int err;

  printf("START\n");

  if ((err = MXC_CLI_Init(MXC_UART_GET_UART(CONSOLE_UART), user_commands,
                          num_user_commands)) != E_NO_ERROR)
  {
    return err;
  }
initSPI();             // Setup SPI
  BIAsettings();         // Set up correct sensor registers for SFBIA
  setFreq(current_freq);
  setupButtonInterrupt();
  setupMax30009Interrupt();
  mxc_tmr_cfg_t tmr0_cfg;
  tmr0_cfg.pres = TMR_PRES_1;
  tmr0_cfg.mode = TMR_MODE_CONTINUOUS;
  tmr0_cfg.bitMode = TMR_BIT_MODE_32;
  tmr0_cfg.clock = MXC_TMR_APB_CLK;
  tmr0_cfg.cmp_cnt = 0xFFFFFFFF;
  tmr0_cfg.pol = 0;

  MXC_TMR_Init(MXC_TMR0, &tmr0_cfg, false);
  MXC_TMR_Start(MXC_TMR0);
  sample_interval_us = getSampleInterval();

  mxc_tmr_cfg_t timer1_cfg;
  timer1_cfg.pres = TMR_PRES_1;
  timer1_cfg.mode = TMR_MODE_CONTINUOUS;
  timer1_cfg.bitMode = TMR_BIT_MODE_32;
  timer1_cfg.clock = MXC_TMR_ISO_CLK;
  timer1_cfg.cmp_cnt = 0xFFFFFFFF;
  timer1_cfg.pol = 0;

  MXC_TMR_Init(MXC_TMR1, &timer1_cfg, false);
  MXC_TMR_Start(MXC_TMR1);
  static uint32_t last_call = 0;
printf("Streaming via INT1…\n");
const double LSB_G = 0.061 / 1000.0;

while (1) {
    if (imu_drdy_flag) {
        
    
    imu_drdy_flag = 0;

    // Service until the line drops (latched level-high semantics)
    do {
        uint8_t raw[6];
        if (i2c_read(OUTX_L_XL, raw, 6) != 0) break;
        (void)i2c_read(STATUS_REG, &st, 1);  // belt-and-suspenders ack

        int16_t ax = (int16_t)((raw[1]<<8)|raw[0]);
        int16_t ay = (int16_t)((raw[3]<<8)|raw[2]);
        int16_t az = (int16_t)((raw[5]<<8)|raw[4]);
        printf("g: X=%.3f Y=%.3f Z=%.3f\n",
               (double)ax*LSB_G, (double)ay*LSB_G, (double)az*LSB_G);
        fflush(stdout);

        // If still asserted (e.g., higher ODR), keep draining
    } while (MXC_GPIO_InGet(IMU_INT_PORT, IMU_INT_MASK));

    // Re-enable GPIO interrupt now that source is cleared
    MXC_GPIO_EnableInt(IMU_INT_PORT, IMU_INT_MASK);
    MXC_Delay(MXC_DELAY_MSEC(1));
    }
    if (interrupt)
    {
      if (sample_ready)
      {
        sample_ready = 0;
        spiBurst(getBiozFreq());

        current_freq = !current_freq;
        setFreq(current_freq);
        interrupt = 0;
        // uint32_t end = MXC_TMR_GetCount(MXC_TMR1);
        // uint32_t delta = end - start;
        // printf("time for burst: %lu cycles\n", delta);
      }
      else 
      {
        sample_index++;
        samples_discarded++;
        spiBurstnoPrint();
        // uint32_t end = MXC_TMR_GetCount(MXC_TMR1);
        // uint32_t delta = end - start;
        interrupt = 0;
        // printf("time for discarded burst: %lu cycles\n", delta);
      }
}

}
}
