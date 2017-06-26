/* Standard includes. */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Tiva Hardware includes. */
#include "inc/tm4c123gh6pm.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/comp.h"
#include "inc/hw_memmap.h"

/* Optional includes for USB serial output */
#ifdef USB_SERIAL_OUTPUT
#include "driverlib/rom.h"
#include "utils/uartstdio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#endif

/*local includes*/
#include "assert.h"
#include "display.h"


// the three tiva LED'a are attached to GPIO
// port F at pind 1, 2, 3
#define LED_R (1<<1)
#define LED_B (1<<2)
#define LED_G (1<<3)

#define LED_ON(x) (GPIO_PORTF_DATA_R |= (x))
#define LED_OFF(x) (GPIO_PORTF_DATA_R &= ~(x))
#define LED(led,on) ((on)?LED_ON(led):LED_OFF(led))


uint32_t  SystemCoreClock;


#ifdef USB_SERIAL_OUTPUT

#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
	_assert_failed ("__error__", pcFilename, ui32Line);
}
#endif

//*****************************************************************************
//
//: Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
static void
_configureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    ROM_UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

#endif


static void
_setupHardware(void)
{
    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    GPIO_PORTF_CR_R = LED_R | LED_G | LED_B ;

    // Enable the GPIO pin for the LED (PF3).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED_R | LED_G | LED_B);

    SystemCoreClock = 80000000;  // Required for FreeRTOS.

    SysCtlClockSet( SYSCTL_SYSDIV_2_5 |
                    SYSCTL_USE_PLL |
                    SYSCTL_XTAL_16MHZ |
                    SYSCTL_OSC_MAIN);

}

static void
_heartbeat( void *notUsed )
{
    uint32_t greenMs = 1000 / portTICK_RATE_MS;
    uint32_t ledOn = 0;
    uint32_t count=0;

    while(1)
    {

        ledOn = !ledOn;

        LED(LED_G, ledOn);

        display_set_cursor(0, 0);
        display_character('A'+count) ;

        count++;
        vTaskDelay(greenMs);
    }
}

int main( void )
{
    _setupHardware();
    display_init();

#ifdef USB_SERIAL_OUTPUT
	void spinDelayMs(uint32_t ms);
	_configureUART();
	spinDelayMs(1000);  // Allow UART to setup
	UARTprintf("\n\nHello from reflow main()\n");
#endif

    unsigned char i = 0;
    unsigned char str_1[] = "REFLOW MANIA" ;
    unsigned char str_2[] = "STARTUP" ;

    display_clear();
    display_set_cursor(0, 4);


    while(str_1[i] != '\0')
    {
        display_character(str_1[i]);
        i++;
    }

    //display_command(0xC2) ;
    display_set_cursor(1, 5);
    i=0;
    
    while(str_2[i] != '\0')
    {
        display_character(str_2[i]);
        i++;
    }

    xTaskCreate(_heartbeat,
                "heartbeat",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY,
                NULL );

    /* Start the tasks and timer running. */
    vTaskStartScheduler();

    assert(0); // we should never get here..

    return 0;
}
