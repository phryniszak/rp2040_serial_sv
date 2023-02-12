/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************
*                                                                    *
*            (c) 1995 - 2021 SEGGER Microcontroller GmbH             *
*                                                                    *
*       www.segger.com     Support: support@segger.com               *
*                                                                    *
**********************************************************************
*                                                                    *
*       SEGGER SystemView * Real-time application analysis           *
*                                                                    *
**********************************************************************
*                                                                    *
* All rights reserved.                                               *
*                                                                    *
* SEGGER strongly recommends to not make any changes                 *
* to or modify the source code of this software in order to stay     *
* compatible with the SystemView and RTT protocol, and J-Link.       *
*                                                                    *
* Redistribution and use in source and binary forms, with or         *
* without modification, are permitted provided that the following    *
* condition is met:                                                  *
*                                                                    *
* o Redistributions of source code must retain the above copyright   *
*   notice, this condition and the following disclaimer.             *
*                                                                    *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND             *
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,        *
* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF           *
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           *
* DISCLAIMED. IN NO EVENT SHALL SEGGER Microcontroller BE LIABLE FOR *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR           *
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT  *
* OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;    *
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF      *
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT          *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE  *
* USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH   *
* DAMAGE.                                                            *
*                                                                    *
**********************************************************************
*                                                                    *
*       SystemView version: 3.42                                    *
*                                                                    *
**********************************************************************
-------------------------- END-OF-HEADER -----------------------------

File    : SEGGER_SYSVIEW_Config_NoOS.c
Purpose : Sample setup configuration of SystemView without an OS for Cortex-M0.
Revision: $Rev: 18540 $
*/
#include <SEGGER_RTT.h>
#include "SEGGER_SYSVIEW.h"
#include "SEGGER_SYSVIEW_Conf.h"

// ph
#include "pico/stdlib.h"
#include "hardware/structs/systick.h"
#include "hardware/structs/scb.h"
#include "hardware/clocks.h"
#include "hardware/exception.h"
#include "pico/platform.h"

/*********************************************************************
 *
 *       Defines, configurable
 *
 **********************************************************************
 */
// The application name to be displayed in SystemViewer
#define SYSVIEW_APP_NAME "Demo Application"

// The target device name
#define SYSVIEW_DEVICE_NAME "Cortex-M0"

// The lowest RAM address used for IDs (pointers)
#define SYSVIEW_RAM_BASE (0x20000000)

/*********************************************************************
 *
 *       Defines, fixed
 *
 **********************************************************************
 */

/*********************************************************************
 *
 *       Static variables
 *
 **********************************************************************
 */

static enum {
    WAIT_FOR_HOST_HELLO1_S,
    WAIT_FOR_HOST_HELLO2_V,
    WAIT_FOR_HOST_HELLO3_VERSION1,
    WAIT_FOR_HOST_HELLO4_VERSION2_SEND_CLIENT_HELLO,
    RECORDING
} _State = WAIT_FOR_HOST_HELLO1_S;

static int _ChannelId = -1;

/*********************************************************************
 *
 *       Private functions
 *
 **********************************************************************
 */

void _on_event_recorded(int num_bytes)
{
    // if fifo empty
    if (uart_get_hw(APP_UART_INTERFACE)->fr & UART_UARTFR_TXFE_BITS)
    {
        // send data in up to 32 byte chunks
        unsigned char data[32];
        unsigned int r = SEGGER_RTT_ReadUpBufferNoLock(_ChannelId, data, count_of(data));
        for (size_t i = 0; i < r; i++)

            uart_get_hw(APP_UART_INTERFACE)->dr = data[i];
    }
}

/*********************************************************************
 *
 *       _process_incoming_char()
 *
 *  Function description
 *    Event handler to be called upon receiving incoming data.
 *
 *  Parameters
 *    Byte   - received char.
 *
 *  Return value
 *    true: hello received
 *    false: no hello received
 */
int _check_read_hello(const unsigned char Byte)
{
    static int lastIsStarted = false;
    int isStarted = SEGGER_SYSVIEW_IsStarted();

    // wait for disconnection event
    if (lastIsStarted && !isStarted)
    {
        // and if so, reset state
        _State = WAIT_FOR_HOST_HELLO1_S;
    }
    lastIsStarted = isStarted;

    switch (_State)
    {
    case RECORDING:
        // 9. end of state machine
        return true;
    case WAIT_FOR_HOST_HELLO4_VERSION2_SEND_CLIENT_HELLO:
        // TODO: check that in fifo is enough space
        // be nice and respond with hello
        uart_get_hw(APP_UART_INTERFACE)->dr = 'S';
        uart_get_hw(APP_UART_INTERFACE)->dr = 'V';
        uart_get_hw(APP_UART_INTERFACE)->dr = SEGGER_SYSVIEW_MAJOR;
        uart_get_hw(APP_UART_INTERFACE)->dr = SEGGER_SYSVIEW_MINOR;
        // 4.
        _State = RECORDING; // we don't care about the clients version
        break;
    case WAIT_FOR_HOST_HELLO3_VERSION1:
        // 3.
        _State = WAIT_FOR_HOST_HELLO4_VERSION2_SEND_CLIENT_HELLO; // we don't care about the clients version
        break;
    case WAIT_FOR_HOST_HELLO2_V:
        // 2.
        if ('V' == Byte)
        {
            _State = WAIT_FOR_HOST_HELLO3_VERSION1;
        }
        else
        {
            _State = WAIT_FOR_HOST_HELLO1_S;
        }
        break;
    case WAIT_FOR_HOST_HELLO1_S:
        // 1.
        if ('S' == Byte)
        {
            _ChannelId = SEGGER_SYSVIEW_GetChannelID();
            _State = WAIT_FOR_HOST_HELLO2_V;
        }
        else
        {
            _State = WAIT_FOR_HOST_HELLO1_S;
        }
        break;
    }

    return false;
}

// UART interrupt handler
void _on_uart_irq()
{
    uint32_t status = uart_get_hw(APP_UART_INTERFACE)->mis;

    // RX
    if (status & (UART_UARTICR_RXIC_BITS | UART_UARTICR_RTIC_BITS))
    {
        while (uart_is_readable(APP_UART_INTERFACE))
        {
            uint8_t ch = (uint8_t)uart_get_hw(APP_UART_INTERFACE)->dr;
            if (_check_read_hello(ch))
            {
                // Write data into corresponding RTT buffer for application to read and handle accordingly
                SEGGER_RTT_WriteDownBuffer(_ChannelId, &ch, 1);
            }
        }
    }

    // TX
    // if (status & UART_UARTICR_TXIC_BITS)
    // {
    // }
}

/**
 * @brief
 *
 */
void _serial_init(void)
{
    // Set up our UART with the required speed.
    uart_init(APP_UART_INTERFACE, APP_UART_BAUDRATE);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(APP_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(APP_UART_RX, GPIO_FUNC_UART);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = APP_UART_INTERFACE == uart0 ? UART0_IRQ : UART1_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, _on_uart_irq);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(APP_UART_INTERFACE, true, false);
}

/**
 * @brief
 *
 */
void __time_critical_func(_systick_handler)(void)
{
    // SEGGER_SYSVIEW_RecordEnterISR();
    SEGGER_SYSVIEW_TickCnt++;
    // SEGGER_SYSVIEW_RecordExitISR();
}

/*********************************************************************
 *
 *       _cbSendSystemDesc()
 *
 *  Function description
 *    Sends SystemView description strings.
 */
static void _cbSendSystemDesc(void)
{
    SEGGER_SYSVIEW_SendSysDesc("N=" SYSVIEW_APP_NAME ",D=" SYSVIEW_DEVICE_NAME);
    SEGGER_SYSVIEW_SendSysDesc("I#15=SysTick");
}

/*********************************************************************
 *
 *       Global functions
 *
 **********************************************************************
 */
void SEGGER_SYSVIEW_Conf(void)
{
    // START serial port
    _serial_init();
    // END serial port

    // below standard SystemView cortex-m0 implementation
    U32 SystemCoreClock = clock_get_hz(clk_sys);
    // Frequency of the timestamp. Must match SEGGER_SYSVIEW_Conf.h
    U32 SYSVIEW_TIMESTAMP_FREQ = SystemCoreClock;

    // System Frequency. SystemCoreClock is used in most CMSIS compatible projects.
    U32 SYSVIEW_CPU_FREQ = SystemCoreClock;

    SEGGER_SYSVIEW_Init(SYSVIEW_TIMESTAMP_FREQ, SYSVIEW_CPU_FREQ, 0, _cbSendSystemDesc);
    SEGGER_SYSVIEW_SetRAMBase(SYSVIEW_RAM_BASE);

    // START systick
    // we put it at the end as it generates interrupt, so better is to start it
    // after everything is else is ready
    systick_hw->rvr = M0PLUS_SYST_RVR_BITS; // max 24 bit value
    systick_hw->cvr = 0;                    /* resets timer to last_load */
    systick_hw->csr |= (M0PLUS_SYST_CSR_ENABLE_BITS |
                        M0PLUS_SYST_CSR_CLKSOURCE_BITS |
                        M0PLUS_SYST_CSR_TICKINT_BITS);

    exception_set_exclusive_handler(SYSTICK_EXCEPTION, _systick_handler);
    // END systick
}

/*********************************************************************
 *
 *       SEGGER_SYSVIEW_X_GetTimestamp()
 *
 * Function description
 *   Returns the current timestamp in ticks using the system tick
 *   count and the SysTick counter.
 *   All parameters of the SysTick have to be known and are set via
 *   configuration defines on top of the file.
 *
 * Return value
 *   The current timestamp.
 *
 * Additional information
 *   SEGGER_SYSVIEW_X_GetTimestamp is always called when interrupts are
 *   disabled. Therefore locking here is not required.
 */
U32 __time_critical_func(SEGGER_SYSVIEW_X_GetTimestamp)(void)
{
#ifndef USE_FAST_TIMESTAMP
    U32 TickCount;
    U32 Cycles;
    U32 CyclesPerTick;
    //
    // Get the cycles of the current system tick.
    // SysTick is down-counting, subtract the current value from the number of cycles per tick.
    //
    CyclesPerTick = systick_hw->rvr + 1;
    Cycles = (CyclesPerTick - systick_hw->cvr);
    //
    // Get the system tick count.
    //
    TickCount = SEGGER_SYSVIEW_TickCnt;
    //
    // If a SysTick interrupt is pending, re-read timer and adjust result
    //
    if ((scb_hw->icsr & M0PLUS_ICSR_PENDSTSET_BITS) != 0)
    {
        Cycles = (CyclesPerTick - systick_hw->cvr);
        TickCount++;
    }
    Cycles += TickCount * CyclesPerTick;
    return Cycles;
#else
    // no monkey business, use counter to the max (24 bit)
    // TODO: doesn't work for now...
    U32 Cycles = (~systick_hw->rvr) & M0PLUS_SYST_RVR_BITS;
    U32 TickCount = SEGGER_SYSVIEW_TickCnt;
    if ((scb_hw->icsr & M0PLUS_ICSR_PENDSTSET_BITS) != 0)
    {
        Cycles = (~systick_hw->rvr) & M0PLUS_SYST_RVR_BITS;
        TickCount++;
    }
    Cycles += TickCount * (M0PLUS_SYST_RVR_BITS + 1);
    return Cycles;
#endif
}

/*********************************************************************
 *
 *       SEGGER_SYSVIEW_X_GetInterruptId()
 *
 * Function description
 *   Return the currently active interrupt Id,
 *   which ist the active vector taken from IPSR[5:0].
 *
 * Return value
 *   The current currently active interrupt Id.
 *
 * Additional information
 *   This function is not used by default, as the active vector can be
 *   read from ICSR instead on Cortex-M0.
 *   For Cortex-M0+ devices, change SEGGER_SYSVIEW_GET_INTERRUPT_ID
 *   in SEGGER_SYSVIEW_Conf.h to call this function instead.
 */
U32 inline SEGGER_SYSVIEW_X_GetInterruptId(void)
{
    return (scb_hw->icsr & M0PLUS_ICSR_VECTACTIVE_BITS);
}

/*************************** End of file ****************************/
