#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "assert.h"

#include "utils/uartstdio.h"


int isIntContext(void)
{
    int res = 0;
    __asm ("mrs    r0, iapsr\n\t"
           "mov    %[result], r0"
           : [result]"=r" (res) /* 'result' is output */
           :                    /* No input. */
           : "r0"               /* r0 was clobbered */
    );

    return (res & 0x000001ff);
}

void
spinDelayUs(uint32_t us)
{
    while(us--)
    {
        __asm("    nop\n"
              "    nop\n"
              "    nop\n"
              "    nop\n"
              "    nop\n"
              "    nop\n"
              "    nop\n"
              "    nop\n"
              "    nop\n"
              "    nop\n"
              "    nop\n"
              "    nop\n");
    }
}

void
spinDelayMs(uint32_t ms)
{
    while(ms--)
    {
        spinDelayUs(1000);
    }
}


void
_assert_failed (const char *assertion, const char *file, unsigned int line)
{
    // Not alot that we can do at the current time so simply blink the
    // LED rapidly
    //
    // Normally an IO would display:
    //   Assertion failed: expression, file filename, line line number

#ifdef USB_SERIAL_OUTPUT
	UARTprintf("\n\nAssertion Failed: (%s) at %s::%d\n", assertion, file, line);
#endif


    if ( ! isIntContext() )
    {
        // disable interrupts and task switching
        taskENTER_CRITICAL();
    }
    taskDISABLE_INTERRUPTS();


    // Denergize any outputs
    //void motorAllOff(void);
    //motorAllOff();


    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF;

    //
    // Do a dummy read to insert a few cycles after enabling the peripheral.
    //
    uint32_t ui32Loop = SYSCTL_RCGC2_R;

    //
    // Enable the GPIO pin for the LED (PF3).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    //
    GPIO_PORTF_DIR_R = (1<<1) | (1<<2);
    GPIO_PORTF_DEN_R = (1<<1) | (1<<2);

    // turn off all the LED's
    //
    GPIO_PORTF_DATA_R &= ~((1<<2) | (1<<1));

    //
    // Loop forever.
    //
    ui32Loop = 700;
    while(1)
    {
        //
        // Flash Fast the LED.
        //
        int cnt=((ui32Loop/2) * (ui32Loop/13) );
        while(cnt--)
        {
            // This block takes about 13.28us
            GPIO_PORTF_DATA_R |= (1<<2);
            spinDelayUs(10);
            GPIO_PORTF_DATA_R &= ~(1<<2);
            spinDelayUs(26);
        }

        cnt=((ui32Loop/2) * (ui32Loop/13) );
        while(cnt--)
        {
            // This block takes about 13.28us
            GPIO_PORTF_DATA_R |= (1<<1);
            spinDelayUs(2);
            GPIO_PORTF_DATA_R &= ~(1<<1);
            spinDelayUs(34);
        }
    }
}


void vApplicationMallocFailedHook( void )
{
    assert(0);
	for( ;; );
}


void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

    assert(0);
	for( ;; );
}
