/**
 * Copyright (c) 2023 Pawel Hryniszak
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"

#include "SEGGER_SYSVIEW.h"
#include "app_config.h"

unsigned int _TestFunc0Cnt;
unsigned int _TestFunc1Cnt;

/**
 * @brief test systick overflow as counter is 24bit what gives
 * 1÷(125000000÷2^24) = 0.134217728 sec period
 */
static void _TestFunc1Sec(void)
{
    SEGGER_SYSVIEW_RecordVoid(35);
    uint64_t start = time_us_64();
    sleep_ms(1000);
    uint32_t diff = time_us_64() - start;
    SEGGER_SYSVIEW_RecordEndCall(35);
    SEGGER_SYSVIEW_PrintfTarget("usec=%d", diff);
}

/*********************************************************************
 *
 *       _TestFunc1()
 *
 *  Function description
 *    Simple dummy function.
 */
static void _TestFunc1(void)
{
    SEGGER_SYSVIEW_RecordVoid(34);
    _TestFunc1Cnt = 100;
    do
    {
        _TestFunc1Cnt--;
    } while (_TestFunc1Cnt);
    SEGGER_SYSVIEW_RecordEndCall(34);
}

/*********************************************************************
 *
 *       _TestFunc0()
 *
 *  Function description
 *    Simple dummy calling _TestFunc1()
 */
static void _TestFunc0(void)
{
    SEGGER_SYSVIEW_RecordVoid(33);
    _TestFunc0Cnt = 100;
    while (50 < --_TestFunc0Cnt)
        ;
    _TestFunc1();
    while (--_TestFunc0Cnt)
        ;
    SEGGER_SYSVIEW_RecordEndCall(33);
}

int main()
{
    SEGGER_SYSVIEW_Conf(); /* Configure and initialize SystemView  */
    // - as we use UART SystemView should be started/stopped by app
    // SEGGER_SYSVIEW_Start();  /* Starts SystemView recording*/
    SEGGER_SYSVIEW_OnIdle(); /* Tells SystemView that System is currently in "Idle"*/
    int something = 123;
    SEGGER_SYSVIEW_PrintfTarget("test %d", something);

#ifndef APP_LED
#warning blink example requires a board with a regular LED
#else
    const uint LED_PIN = APP_LED;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    while (true)
    {
        sleep_ms(10);
        _TestFunc0();
        gpio_put(LED_PIN, SEGGER_SYSVIEW_IsStarted());
        sleep_ms(10);
        _TestFunc0();
        sleep_ms(10);
        _TestFunc1Sec();
    }
#endif
}