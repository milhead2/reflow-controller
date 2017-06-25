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


#define LCD_D7 (1<<3)  // D3
#define LCD_D6 (1<<2)  // D2
#define LCD_D5 (1<<1)  // D1
#define LCD_D4 (1<<1)  // E1
#define LCD_E  (1<<2)  // E2
#define LCD_RW (1<<4)  // E4
#define LCD_RS (1<<5)  // E5

#define LCD_STROBE {GPIO_PORTE_DATA_R |= LCD_E; GPIO_PORTE_DATA_R &= ~LCD_E; }

#define SET_LCD_RS(x) ((x) ? (GPIO_PORTE_DATA_R |= LCD_RS) : (GPIO_PORTE_DATA_R &= ~(LCD_RS)))
#define SET_LCD_RW(x) ((x) ? (GPIO_PORTE_DATA_R |= LCD_RW) : (GPIO_PORTE_DATA_R &= ~(LCD_RW)))

#define SET_LCD_D4(x) ((x) ? (GPIO_PORTE_DATA_R |= LCD_D4) : (GPIO_PORTE_DATA_R &= ~(LCD_D4)))
#define SET_LCD_D5(x) ((x) ? (GPIO_PORTD_DATA_R |= LCD_D5) : (GPIO_PORTD_DATA_R &= ~(LCD_D5)))
#define SET_LCD_D6(x) ((x) ? (GPIO_PORTD_DATA_R |= LCD_D6) : (GPIO_PORTD_DATA_R &= ~(LCD_D6)))
#define SET_LCD_D7(x) ((x) ? (GPIO_PORTD_DATA_R |= LCD_D7) : (GPIO_PORTD_DATA_R &= ~(LCD_D7)))

#define SET_LCD_EN(x) ((x) ? (GPIO_PORTA_DATA_R |= LCD_E) : (GPIO_PORTA_DATA_R &= ~(LCD_E)))

void SET_LCD_DATA(char val)
{
    // LSB --> D4 --> E1
    SET_LCD_D4( val & (1<<0) );
    // 1   --> D5 --> E1
    SET_LCD_D5( val & (1<<1) );
    // 2   --> D6 --> E1
    SET_LCD_D6( val & (1<<2) );
    // 3   --> D7 --> E1
    SET_LCD_D7( val & (1<<3) );
}

void lcddata(char dataout)
{
    SET_LCD_DATA(dataout);
    SET_LCD_RS(1);
    //RA0 = 1 ;
    SET_LCD_RW(0) ; //PORTA&=~(1<<rw);
    LCD_STROBE ;
}

void lcdcmd(char cmdout)
{
    SET_LCD_EN(0) ;
    SET_LCD_DATA(cmdout);
    SET_LCD_RS(0) ; //PORTA&=~(1<<rs);
    SET_LCD_RW(0) ; //PORTA&=~(1<<rw);
    LCD_STROBE ;
}


void dis_cmd(char cmd_value)
{
    char cmd_value1;
    cmd_value1 = ((cmd_value >> 4) & 0x0F) ; //mask lower nibble because RD0-RD3 pins are used. 
    lcdcmd(cmd_value1); // send to LCD
     
    spinDelayUs(50) ;
    cmd_value1 = (cmd_value & 0x0F) ; // mask higher nibble
    lcdcmd(cmd_value1); // send to LCD
    spinDelayUs(50) ;
}

void dis_data(char data_value)
{
    char data_value1;
    data_value1 = ((data_value >> 4) & 0x0F) ;
    lcddata(data_value1);
    spinDelayUs(50) ;
    data_value1 = (data_value & 0x0F) ;
    lcddata(data_value1);
    spinDelayUs(50) ;
}
 
void _lcdInit(void)
{
    SET_LCD_RS(0); // write control bytes
    spinDelayMs(15) ; // DelayMs(15); // power on delay
    SET_LCD_DATA(0x3); // attention!
    LCD_STROBE;
    spinDelayMs(5) ; // DelayMs(5);
    LCD_STROBE;
    spinDelayUs(100) ; // DelayUs(100);
    LCD_STROBE;
    spinDelayMs(5) ; // DelayMs(5);
    SET_LCD_DATA(0x2); // set 4 bit mode
    LCD_STROBE;
    spinDelayUs(40) ; // DelayUs(40);
    // dis_cmd(0x01) ;
    // dis_cmd(0x02) ;
    dis_cmd(0x28); //to initialize LCD in 2 lines, 5X7 dots and 4bit mode.
    spinDelayMs(5) ;
    dis_cmd(0x0F);
    spinDelayMs(5) ;
    dis_cmd(0x06);
    dis_cmd(0x80);
    spinDelayMs(5) ;
}
 
 
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
    // Ports to dialog with the LCD display
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, LCD_D7 | LCD_D6 | LCD_D5);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, LCD_D4 | LCD_E | LCD_RW | LCD_RS);



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

    while(1)
    {

        ledOn = !ledOn;

        LED(LED_G, ledOn);

        if (ledOn)
            dis_cmd(0x80) ;//+i) ;
        else
            dis_data('A') ;

        vTaskDelay(greenMs);
    }
}

int main( void )
{
    _setupHardware();

#ifdef USB_SERIAL_OUTPUT
	void spinDelayMs(uint32_t ms);
	_configureUART();
	spinDelayMs(1000);  // Allow UART to setup
	UARTprintf("\n\nHello from producerConsumer main()\n");
#endif

    unsigned char i = 0;
    unsigned char str_1[12] = "ELECTRONICS" ;
    unsigned char ch = 'T' , ch_2 = 'E', ch_3 = 'S', ch_4 = 'T' ;

    _lcdInit();

    while(str_1[i] != '\0')
    {
        dis_data(str_1[i]);
        spinDelayMs(5000);
        i++;
    }

    dis_cmd(0xC2) ;
    spinDelayMs(5000) ;
    dis_data(ch) ;
    spinDelayMs(5000) ;
    dis_data(ch_2) ;
    spinDelayMs(5000) ;
    dis_data(ch_3) ;
    spinDelayMs(5000) ;
    dis_data(ch_4) ;
    spinDelayMs(5000) ;

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
