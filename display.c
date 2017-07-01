/* Standard includes. */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

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

#define SET_LCD_EN(x) ((x) ? (GPIO_PORTE_DATA_R |= LCD_E) : (GPIO_PORTE_DATA_R &= ~(LCD_E)))


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
    SET_LCD_RW(0) ; //PORTA&=~(1<<rw);
    LCD_STROBE ;
}

void lcdcmd(char cmdout)
{
    SET_LCD_DATA(cmdout);
    SET_LCD_RS(0) ; //PORTA&=~(1<<rs);
    SET_LCD_RW(0) ; //PORTA&=~(1<<rw);
    LCD_STROBE ;
}


void display_command(char cmd_value)
{
    char cmd_value1;
    cmd_value1 = ((cmd_value >> 4) & 0x0F) ; //mask lower nibble because RD0-RD3 pins are used. 
    lcdcmd(cmd_value1); // send to LCD
     
    spinDelayMs(1) ;
    cmd_value1 = (cmd_value & 0x0F) ; // mask higher nibble
    lcdcmd(cmd_value1); // send to LCD
    spinDelayMs(1) ;
}

void display_character(char data_value)
{
    char data_value1;
    data_value1 = ((data_value >> 4) & 0x0F) ;
    lcddata(data_value1);
    spinDelayMs(1) ;
    data_value1 = (data_value & 0x0F) ;
    lcddata(data_value1);
    spinDelayMs(1) ;
}
 
void _lcdInit(void)
{
    uint8_t cursor_on = 0;
    uint8_t cursor_blink = 0;
    uint8_t display_on = 1;

    //
    // WAKE sequence
    //
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
    display_command(0x28); //to initialize LCD in 2 lines, 5X7 dots and 4bit mode.
    spinDelayMs(5) ;

    // Display ON/OFF Control
    display_command(0x08 | (display_on << 2) | (cursor_on << 1) | (cursor_blink <<0)); //0x0F);
    spinDelayMs(5) ;


    display_command(0x06);
    display_command(0x80);
    spinDelayMs(10) ;
}
 
 


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

}


void display_init(void)
{
    _setupHardware();
    _lcdInit();
}

void display_clear(void)
{
    display_command(0x01);
    spinDelayMs(10) ;
}

void display_home(void)
{
    display_command(0x02);
    spinDelayMs(10) ;
}

void display_set_cursor(int line, int offset)
{
    uint8_t addr=offset;
    addr |= (line) ? 0xC0 : 0x80;
    display_command(addr);
    spinDelayMs(1) ;
}

void display_string(const char * s)
{
    while(*s)
        display_character(*(s++));
}

void display_clear_line(int line)
{
    if (line)
    {
        display_set_cursor(0,0);
        display_string("                ");
    }
    else
    {    
        display_set_cursor(1,0);
        display_string("                ");
    }
}

#define PBUFF_MAX 32
char pbuffer[PBUFF_MAX];
static SemaphoreHandle_t _semBuf = NULL;

#if 0
extern caddr_t _sbrk(int incr)
{
	return (caddr_t) 0;
}
#endif

void display_printf(int line, int offset, const char *format, ...)
{
    va_list vaArgP;

    if (_semBuf && (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING))
    {
        xSemaphoreTake( _semBuf, portMAX_DELAY);
    }
    else
    {
        if (! _semBuf) 
        {
            _semBuf = xSemaphoreCreateBinary();
            xSemaphoreGive( _semBuf );
            assert(_semBuf);
        }
    }

    va_start(vaArgP, format);
    vsnprintf(pbuffer, PBUFF_MAX, format, vaArgP);
    //sprintf(pbuffer, format);
    display_set_cursor(line, offset);
    display_string(pbuffer);
    va_end(vaArgP);

    if (_semBuf && (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING))
    {
        xSemaphoreGive( _semBuf );
    }
}


