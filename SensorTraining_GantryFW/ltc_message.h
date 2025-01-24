#ifndef _ltc_message
#define _ltc_message

#include <stdint.h>

//what comes out of one channel for the LTC chip
struct conversion_unit{
    uint8_t cnv_upper; // two's compliment --> total is 2's compliment
    uint8_t cnv_lower;
    uint8_t info;
    };

// 24bytes / channel * 8 channels = 192 bytes
// 24 8-bit words
struct ltc_spi
{
    conversion_unit channel[6]; //[8]; //8 channel ADC
};



#endif