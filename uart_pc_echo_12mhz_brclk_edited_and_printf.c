/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/******************************************************************************
 * MSP432 UART - PC Echo with 12MHz BRCLK
 *
 * Description: This demo echoes back characters received via a PC serial port.
 * SMCLK/DCO is used as a clock source and the device is put in LPM0
 * The auto-clock enable feature is used by the eUSCI and SMCLK is turned off
 * when the UART is idle and turned on when a receive edge is detected.
 * Note that level shifter hardware is needed to shift between RS232 and MSP
 * voltage levels.
 *
 * Edited version:
 * Toggles the Red LED if the character received is an "L".
 * Also adds a "." after the character when it sends it back to the PC.
 *
 *               MSP432P401
 *             -----------------
 *            |                 |
 *            |                 |
 *            |                 |
 *       RST -|     P1.3/UCA0TXD|----> PC (echo)
 *            |                 |
 *            |                 |
 *            |     P1.2/UCA0RXD|<---- PC
 *            |                 |
 *
 *******************************************************************************/
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
//#include <stdio.h>  // Only include if you want to have output to the Console.
#include "printf.h"

static volatile uint8_t readdata;

//![Simple UART Config]
/* UART Configuration Parameter. These are the configuration parameters to
 * make the eUSCI A UART module to operate with a 9600 baud rate. These
 * values were calculated using the online calculator that TI provides
 * at:
 *http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
 */
const eUSCI_UART_Config uartConfig =
{
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        78,                                     // BRDIV = 78
        2,                                       // UCxBRF = 2
        0,                                       // UCxBRS = 0
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_LSB_FIRST,                  // LSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
};
//![Simple UART Config]

int main(void)
{
    /* Halting WDT  */
    MAP_WDT_A_holdTimer();

    /* Selecting P1.2 and P1.3 in UART mode */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
            GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    // Use the LED to indicate if we got an "L" through the terminal
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

//    /* Setting DCO to 12MHz */
//    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);
    // Alternative which is better:
    CS_setDCOFrequency(12000000);

    //![Simple UART Example]
    /* Configuring UART Module */
    MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig);

    /* Enable UART module */
    MAP_UART_enableModule(EUSCI_A0_BASE);

    /* Enabling interrupts */
    MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA0);
    MAP_Interrupt_enableSleepOnIsrExit();
    MAP_Interrupt_enableMaster();   
    //![Simple UART Example]


    while(1)
    {
        MAP_PCM_gotoLPM0();
    }
}

/* EUSCI A0 UART ISR - Echoes data back to PC host */
void EUSCIA0_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE);

    MAP_UART_clearInterruptFlag(EUSCI_A0_BASE, status);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        readdata = MAP_UART_receiveData(EUSCI_A0_BASE);

        MAP_UART_transmitData(EUSCI_A0_BASE, readdata);
        MAP_UART_transmitData(EUSCI_A0_BASE,'.');

        // Toggle Red LED if the character received is an "L":
        if (readdata == 'L') {  // 'L' is 76 in the ASCII table
        // if (readdata == 76) {  // This also works.

            MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);

            const char mytext[] = "\n\rToggle LED\n\r";
            int ichar;
            for (ichar=0; ichar<14; ichar++) {
                MAP_UART_transmitData(EUSCI_A0_BASE,mytext[ichar]);
            }

            char *s = "printf test";
            char c = '!';
            int i = -12345;
            unsigned u = 4321;
            long int l = -123456780;
            long unsigned n = 1098765432;
            unsigned x = 0xABCD;

            // DOES NOT WORK, must use EUSCI_A0_BASE:
            //printf(EUSCI_A0_MODULE, "String         %s\r\n", s);
            // This works:
            // Also must #include "printf.h" at the top of the file, and put
            // printf.h, printf.c as part of the project.
            printf(EUSCI_A0_BASE, "String         %s\r\n", s);
            printf(EUSCI_A0_BASE, "Char           %c\r\n", c);
            printf(EUSCI_A0_BASE, "Integer        %i\r\n", i);
            printf(EUSCI_A0_BASE, "Unsigned       %u\r\n", u);
            printf(EUSCI_A0_BASE, "Long           %l\r\n", l);
            printf(EUSCI_A0_BASE, "uNsigned loNg  %n\r\n", n);
            printf(EUSCI_A0_BASE, "heX            %x\r\n", x);

            // This output goes to the CONSOLE, NOT THE TERMINAL:
            // It is slow because it uses JTAG
            // Must also #include <stdio.h> at the top of the file to get it to work.
            // printf("Printed!\n\r");
            // printf("%d\n\r",12345);
        }

    }

}

