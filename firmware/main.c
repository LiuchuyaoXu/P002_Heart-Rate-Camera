/*
    File:       main.c

    Author:     Liuchuyao Xu, 2019
*/

#include "board.h"
#include "clock_config.h"
#include "peripherals.h"
#include "pin_mux.h"

#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_i2c.h"
#include "fsl_lpuart.h"
#include "fsl_port.h"

#define UART                LPUART0
#define UART_BAUDRATE       9600
#define UART_CLK_SRC        SYS_CLK
#define UART_CLK_FREQ       CLOCK_GetFreq(UART_CLK_SRC)

#define I2C                     I2C0
#define I2C_CLK_SRC             I2C0_CLK_SRC
#define I2C_CLK_FREQ            CLOCK_GetFreq(I2C_CLK_SRC)
#define I2C_BAUDRATE            100000
#define I2C_SLAVE_ADDR          0x1D
#define I2C_SLAVE_STATUS_ADDR   0x00
#define I2C_SlAVE_CTRL_ADDR     0X2A
#define I2C_XYZ_DATA_CFG_ADDR   0x0E

#define I2C_RELEASE_SDA_PORT    PORTB
#define I2C_RELEASE_SCL_PORT    PORTB
#define I2C_RELEASE_SDA_GPIO    GPIOB
#define I2C_RELEASE_SCL_GPIO    GPIOB
#define I2C_RELEASE_SDA_PIN     4U
#define I2C_RELEASE_SCL_PIN     3U
#define I2C_RELEASE_BUS_COUNT   100U

i2c_master_handle_t i2cHandle;

volatile bool i2cCompletionFlag = false;
volatile bool i2cNakFlag        = false;

static void i2c_release_bus_delay(void)
{
    uint32_t i = 0;
    for (i = 0; i < I2C_RELEASE_BUS_COUNT; i++)
    {
        __NOP();
    }
}

void BOARD_I2C_ReleaseBus(void)
{
    uint8_t i = 0;
    gpio_pin_config_t pin_config;
    port_pin_config_t i2c_pin_config = {0};

    /* Config pin mux as gpio */
    i2c_pin_config.pullSelect = kPORT_PullUp;
    i2c_pin_config.mux = kPORT_MuxAsGpio;

    pin_config.pinDirection = kGPIO_DigitalOutput;
    pin_config.outputLogic = 1U;
    CLOCK_EnableClock(kCLOCK_PortB);
    PORT_SetPinConfig(I2C_RELEASE_SCL_PORT, I2C_RELEASE_SCL_PIN, &i2c_pin_config);
    PORT_SetPinConfig(I2C_RELEASE_SDA_PORT, I2C_RELEASE_SDA_PIN, &i2c_pin_config);

    GPIO_PinInit(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, &pin_config);
    GPIO_PinInit(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, &pin_config);

    /* Drive SDA low first to simulate a start */
    GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    /* Send 9 pulses on SCL and keep SDA high */
    for (i = 0; i < 9; i++)
    {
        GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
        i2c_release_bus_delay();

        GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
        i2c_release_bus_delay();

        GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
        i2c_release_bus_delay();
        i2c_release_bus_delay();
    }

    /* Send stop */
    GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
    i2c_release_bus_delay();

    GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
    i2c_release_bus_delay();
}

static void i2cCallback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData)
{
    if (status == kStatus_Success) {

        i2cCompletionFlag = true;
    }

    if ((status == kStatus_I2C_Nak) || (status == kStatus_I2C_Addr_Nak)) {

        i2cNakFlag = true;
    }
}

static bool i2cWrite(I2C_Type *base, uint8_t slaveAddr, uint8_t regAddr, uint8_t data)
{
    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress     = slaveAddr;
    masterXfer.direction        = kI2C_Write;
    masterXfer.subaddress       = regAddr;
    masterXfer.subaddressSize   = 1;
    masterXfer.data             = &data;
    masterXfer.dataSize         = 1;
    masterXfer.flags            = kI2C_TransferDefaultFlag;
    I2C_MasterTransferNonBlocking(I2C, &i2cHandle, &masterXfer);

    while((!i2cNakFlag) && (!i2cCompletionFlag)) {

    }

    i2cNakFlag = false;

    if(i2cCompletionFlag == true) {

        i2cCompletionFlag = false;
        return true;
    }
    else {

        return false;
    }
}

static bool i2cRead(I2C_Type *base, uint8_t slaveAddr, uint8_t regAddr, uint8_t *rxBuffer, uint32_t rxSize)
{
    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress     = slaveAddr;
    masterXfer.direction        = kI2C_Read;
    masterXfer.subaddress       = regAddr;
    masterXfer.subaddressSize   = 1;
    masterXfer.data             = rxBuffer;
    masterXfer.dataSize         = rxSize;
    masterXfer.flags            = kI2C_TransferDefaultFlag;
    I2C_MasterTransferNonBlocking(I2C, &i2cHandle, &masterXfer);

    while((!i2cNakFlag) && (!i2cCompletionFlag)) {

    }

    i2cNakFlag = false;

    if(i2cCompletionFlag == true) {

        i2cCompletionFlag = false;
        return true;
    }
    else {

        return false;
    }
}

int main(void)
{
    BOARD_InitPins();
    BOARD_BootClockRUN();
    // BOARD_I2C_ReleaseBus();
    BOARD_I2C_ConfigurePins();
    BOARD_InitDebugConsole();
    CLOCK_SetLpuart0Clock(0x1U);
    
    lpuart_config_t uartConfig;
    LPUART_GetDefaultConfig(&uartConfig);
    uartConfig.baudRate_Bps = UART_BAUDRATE;
    uartConfig.enableTx     = true;
    LPUART_Init(UART, &uartConfig, UART_CLK_FREQ);

    i2c_master_config_t masterConfig;
    I2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = I2C_BAUDRATE;
    I2C_MasterInit(BOARD_ACCEL_I2C_BASEADDR, &masterConfig, I2C_CLK_FREQ);

    // uint8_t startMessage[] = "Program starts.\n\r";
    // LPUART_WriteBlocking(UART, startMessage, sizeof(startMessage) / sizeof(startMessage[0]));
    
    I2C_MasterTransferCreateHandle(I2C, &i2cHandle, i2cCallback, NULL);
    i2cWrite(I2C, I2C_SLAVE_ADDR, I2C_SlAVE_CTRL_ADDR , 0x00);
    i2cWrite(I2C, I2C_SLAVE_ADDR, I2C_XYZ_DATA_CFG_ADDR , 0x01);
    i2cWrite(I2C, I2C_SLAVE_ADDR, I2C_SlAVE_CTRL_ADDR , 0x0d);
    while(1) {

        uint8_t status = 0;
        while(status != 0xff) {
            i2cRead(I2C, I2C_SLAVE_ADDR, I2C_SLAVE_STATUS_ADDR, &status, 1);
        }

        int16_t x, y, z;
        uint8_t readBuffer[7];
        i2cRead(I2C, I2C_SLAVE_ADDR, I2C_SLAVE_STATUS_ADDR, readBuffer, 7);
        status = readBuffer[0];
        x = ((int16_t)(((readBuffer[1] * 256) | readBuffer[2]))) / 4;
        y = ((int16_t)(((readBuffer[3] * 256) | readBuffer[4]))) / 4;
        z = ((int16_t)(((readBuffer[5] * 256) | readBuffer[6]))) / 4;

        uint32_t accel = x*x + y*y + z*z;
        if(accel > 8000000) {
           PRINTF("x"); 
        }

        // PRINTF("status = 0x%x , x = %5d , y = %5d , z = %5d \r\n", status, x, y, z);
        // uint8_t debugMessage[] = "DEBUG MESSAGE.\n\r";
        // LPUART_WriteBlocking(UART,  debugMessage, sizeof(debugMessage) / sizeof(debugMessage[0]));
    }
}
