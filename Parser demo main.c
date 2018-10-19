
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

#include "printf.h"
// project must contain, at the same level as this main.c file,
// the printf.c and printf.h files from the online library;
// they are available on canvas.

// state for the double command
volatile bool handlingDoubleCommand;
volatile int16_t parsedValue;


// halts the watchdog timer
void disableWDT(){
    WDT_A_holdTimer();
}

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

// initializes EUSCI_A0 for UART (with pins 1.2 & 1.3, and the associated receive interrupt)
// doesnt enable transmit interrupt anticipating the use of the printf library from online, which busy-waits.
void configUART(){
    UART_initModule(EUSCI_A0_BASE, &uartConfig);

    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
                GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    UART_enableModule(EUSCI_A0_BASE);

    UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    Interrupt_enableInterrupt(INT_EUSCIA0);
}

// Duplicating UART lines for EUSCI_A0 for Logic Analyzer
const uint8_t port1_mapping[] =
{
        PMAP_NONE,      PMAP_NONE,      PMAP_UCA0RXD,      PMAP_UCA0TXD,
        PMAP_NONE,      PMAP_NONE,      PMAP_NONE,   PMAP_NONE
};

const uint8_t port2_mapping[] =
{
        //Port P2:
        PMAP_NONE,      PMAP_NONE,      PMAP_NONE,      PMAP_NONE,
        PMAP_NONE,      PMAP_NONE,      PMAP_NONE,   PMAP_UCA0TXD
};

// Duplicates EUSCI_A0 TX line onto P2.7  while leaving P1.2 and P1.3 intact
void duplicateUARTOutput(){
    PMAP_configurePorts(port1_mapping, PMAP_P1MAP, 2,
                        PMAP_ENABLE_RECONFIGURATION);

    PMAP_configurePorts(port2_mapping, PMAP_P2MAP, 1,
                PMAP_DISABLE_RECONFIGURATION);

    /* Selecting P1.2 and P1.3, also P2.6 and 2.7 in UART mode */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
            GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2,
                GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);
}

int main(void)
{
    disableWDT();

    CS_setDCOFrequency(12000000); //12MHz DCO & CPU
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1); //12Mhz SM Clock from DCO

    configUART();

    duplicateUARTOutput();

    Interrupt_enableMaster();

    while(1)
    {
        MAP_PCM_gotoLPM0();
    }
}

/* EUSCI A0 UART ISR - Echoes data back to PC host */
void EUSCIA0_IRQHandler(void)
{
    uint32_t status = UART_getEnabledInterruptStatus(EUSCI_A0_BASE);
    UART_clearInterruptFlag(EUSCI_A0_BASE, status);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        volatile char readdata;
        readdata = UART_receiveData(EUSCI_A0_BASE);

        //Transition 1: Start of Double command: (d received while not handlingDoubleCommand)
        if(!handlingDoubleCommand & readdata == 'd'){
            handlingDoubleCommand = true;
            parsedValue = 0;
        }
        else if (handlingDoubleCommand){
            //Transition 2: numeric char received while handlingDoubleCommand:
            if(readdata >= '0' & readdata <= '9'){
                int16_t newdigit = readdata - '0';
                parsedValue = parsedValue * 10 + newdigit;
            } //Transition 3: non-numeric char received while handlingDoubleCommand:
            else {
                handlingDoubleCommand = false;
                printf(EUSCI_A0_BASE, "\nDoubled Value: %i\n", 2*parsedValue);
                parsedValue = 0;
            }
        }
        //Transition 4: unrecognized char received while not handlingDoubleCommand:
        else { // readdata != 'd' and !handlingDoubleCommand
            // deviation from state machine in class: echoing the character back with a ? to indicate an unrecognized command.
            printf(EUSCI_A0_BASE, "%c?\n", readdata);
        }
    }

}

