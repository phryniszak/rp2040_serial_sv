#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

// #if false
// #define picoprobe_info(format,args...) printf(format, ## args)
// #else
// #define picoprobe_info(format,...) ((void)0)
// #endif

// #if false
// #define picoprobe_debug(format,args...) printf(format, ## args)
// #else
// #define picoprobe_debug(format,...) ((void)0)
// #endif

// #if false
// #define picoprobe_dump(format,args...) printf(format, ## args)
// #else
// #define picoprobe_dump(format,...) ((void)0)
// #endif

#ifndef PICO_DEFAULT_LED_PIN
#error PICO_DEFAULT_LED_PIN is not defined, run APP_LED=<led_pin> cmake
#elif PICO_DEFAULT_LED_PIN == -1
#error PICO_DEFAULT_LED_PIN is defined as -1, run APP_LED=<led_pin> cmake
#else
#define APP_LED PICO_DEFAULT_LED_PIN
#endif

#endif
