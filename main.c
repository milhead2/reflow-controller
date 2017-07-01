/* Standard includes. */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

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
#include "profile.h"


// the three tiva LED'a are attached to GPIO
// port F at pind 1, 2, 3
#define LED_R (1<<1)
#define LED_B (1<<2)
#define LED_G (1<<3)

#define LED_ON(x) (GPIO_PORTF_DATA_R |= (x))
#define LED_OFF(x) (GPIO_PORTF_DATA_R &= ~(x))
#define LED(led,on) ((on)?LED_ON(led):LED_OFF(led))

//Port C settings, Interface button
#define BUTTON (1<<5)
#define PHA    (1<<6)
#define PHB    (1<<7)
#define BUTTON_STATE ((GPIO_PORTC_DATA_R & BUTTON) == 0)


uint32_t  SystemCoreClock;
int32_t  _encoder;
const int8_t lookup[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

static void 
_interruptHandlerPortC(void)
{
    uint32_t mask = GPIOIntStatus(GPIO_PORTC_BASE, 1);
    static uint8_t tt=0;

    tt <<= 2; // Shift tt history Left

    // set lower two bits of tt to value of A and B
    uint8_t Port = GPIO_PORTC_DATA_R;
    tt |= (Port & PHB) ? 0x02 : 0x00;
    tt |= (Port & PHA) ? 0x01 : 0x00;

    tt &= 0x0f;

    _encoder += lookup[tt];

    // MAX_INT is a used location.
    if (_encoder == INT_MAX) 
        _encoder = INT_MAX-1;

#if 0
    if ((tt == 0x0001) || (tt == 0x1011))
    {
        uint32_t tNow = _readTimer();
        if (_priorTimer < tNow)
            _velocitySamples[_velocityNext] =  tNow - _priorTimer;
        else
            _velocitySamples[_velocityNext] =  (tNow+0x7fffffff) - (_priorTimer-0x7fffffff);

        _priorTimer = tNow;
        _velocityNext = (_velocityNext+1) % VELO_SAMPLES;
    }
#endif

    GPIOIntClear(GPIO_PORTC_BASE, mask);
}






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
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

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

    //
    // setup buttin press and encoder states
    //
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, (BUTTON | PHA | PHB));
    GPIOPadConfigSet(GPIO_PORTC_BASE, (BUTTON|PHA|PHB), GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOIntRegister(GPIO_PORTC_BASE, _interruptHandlerPortC);
    GPIOIntTypeSet(GPIO_PORTC_BASE, (PHA|PHB), GPIO_BOTH_EDGES);
    GPIOIntEnable(GPIO_PORTC_BASE, (PHA|PHB));
}

uint32_t temp_set;
uint32_t temp_measured;
typedef enum {NO_MODE, REFLOW_MODE, PROFILE_MODE, START_MODE, STARTUP_MODE} operating_mode_t;
typedef struct {
    char * name;
    operating_mode_t mode;
} menu_entry_t;

menu_entry_t menu[] = {
    {"STARTUP MODE", STARTUP_MODE},
    {"PROFILE SELECT", STARTUP_MODE},
    {"START", START_MODE}
};

#define MENU_MAX (sizeof(menu)/sizeof(menu_entry_t))


static operating_mode_t _startup_update(void)
{
    static int last_encoder = INT_MAX;
    static int menu_index = 0;

    display_printf(0, 0, "CHOOSE");

    if (_encoder > last_encoder)
        menu_index = (menu_index + 1) % MENU_MAX;
    last_encoder = _encoder;
    
    display_printf(1, 0, "%s", menu[menu_index].name);

    if (BUTTON_STATE)
    {
        return menu[menu_index].mode;
    }
    else
    {
        return STARTUP_MODE;
    }
}


static operating_mode_t _start_update(void)
{
display_printf(0, 0, "REFLOW READY");

if ((_encoder&1) != 0)
{
    display_printf(1, 0, "BUTTON=CANCEL  ");
    if (BUTTON_STATE)
    {
        display_clear();
        display_printf(0, 0, "CANCELING...");
        vTaskDelay(1000);
        return STARTUP_MODE;
    }
}
else
{
    display_printf(1, 0, "BUTTON=START   ");
    if (BUTTON_STATE)
    {
        display_clear();
        display_printf(0, 0, "STARTING");
        display_printf(1, 0, "REFLOW SEQUENCE");
        vTaskDelay(1000);
        return REFLOW_MODE;
    }
}

return START_MODE;
}


static operating_mode_t _reflow_update(void)
{
uint32_t cycleMs = 100 / portTICK_RATE_MS;
profile_select(PROFILE_LEAD);

TickType_t start_tick = xTaskGetTickCountFromISR();

while(1)
{
    TickType_t ms = xTaskGetTickCountFromISR();
    uint32_t sec = (ms-start_tick) / 1000;
    display_printf(1, 0, "%03d", sec%1000);
    display_printf(0, 0, "%-7s", profile_phase(sec)); 
    display_printf(0, 8, "set:%3dC:", profile_set(sec));
    display_printf(1, 7, "temp:%3dC:", sec);

    if (BUTTON_STATE)
    {
        display_clear();
        display_printf(0, 0, "CANCELING...");
        vTaskDelay(1000);
        return STARTUP_MODE;
    }

    if (profile_set(sec) == 0)
    {
        display_clear();
        display_printf(0, 0, "REFLOW COMPLETE");
        vTaskDelay(2000);
        return STARTUP_MODE;
    }

    vTaskDelay(cycleMs);
}
}


static void
_reflow( void *notUsed )
{
    uint32_t cycleMs = 100 / portTICK_RATE_MS;
    bool reflow_mode = false;

    operating_mode_t mode = START_MODE;
    operating_mode_t prior_mode = NO_MODE;

    vTaskDelay(2000 / portTICK_RATE_MS);

    while(true)
    {
        if (mode != prior_mode)
        {
            display_clear();
            UARTprintf("New Mode: %d\n", mode);
            prior_mode = reflow_mode;

        }

        switch(mode)
        {
            case STARTUP_MODE:
                mode = _startup_update();
                break;

            case START_MODE:
                mode = _start_update();
                break;

            case REFLOW_MODE:
                mode = _reflow_update();
                break;

            case NO_MODE:
            default:
                assert(0);
        }

        vTaskDelay(cycleMs);
    }

}

static void
_heartbeat( void *notUsed )
{
    uint32_t greenMs = 1000 / portTICK_RATE_MS;
    uint32_t ledOn = 0;

    while(1)
    {
        ledOn = !ledOn;
        LED(LED_G, ledOn);
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


    display_clear();
    display_printf(0, 2, "REFLOW MANIA");
    display_printf(1, 4, "STARTUP");

    xTaskCreate(_reflow,
                "heartbeat",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL );

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
