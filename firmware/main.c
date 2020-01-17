/*
    File:       main.c

    Author:     Liuchuyao Xu, 2019
*/

#include "board.h"
#include "clock_config.h"
#include "peripherals.h"
#include "pin_mux.h"

#include "fsl_lpuart.h"

#define UART                LPUART0
#define UART_BAUDRATE       9600
#define UART_CLK_SRC        SYS_CLK
#define UART_CLK_FREQ       CLOCK_GetFreq(UART_CLK_SRC)

int main(void)
{
    BOARD_InitPins();
    BOARD_BootClockRUN();
    CLOCK_SetLpuart0Clock(0x1U);
    
    lpuart_config_t uartConfig;
    LPUART_GetDefaultConfig(&uartConfig);
    uartConfig.baudRate_Bps = UART_BAUDRATE;
    uartConfig.enableTx     = true;
    LPUART_Init(UART, &uartConfig, UART_CLK_FREQ);

    uint8_t startMessage[] = "Program starts.\n\r";
    LPUART_WriteBlocking(UART, startMessage, sizeof(startMessage) / sizeof(startMessage[0]));
    
    while(1) {
        uint8_t debugMessage[] = "x\n\r";
        LPUART_WriteBlocking(UART,  debugMessage, sizeof(debugMessage) / sizeof(debugMessage[0]));
    }
}
