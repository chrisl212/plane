#ifndef TSL2591_H
#define TSL2591_H

#include <stdint.h>

#define TSL2591_COMMAND_BIT 0xA0
#define TSL2591_LUX_DF (408.0)

enum {
    TSL2591_INTEGRATIONTIME_100MS = 0x00,
    TSL2591_INTEGRATIONTIME_200MS,
    TSL2591_INTEGRATIONTIME_300MS,
    TSL2591_INTEGRATIONTIME_400MS,
    TSL2591_INTEGRATIONTIME_500MS,
    TSL2591_INTEGRATIONTIME_600MS
};

enum {
    TSL2591_GAIN_LOW = 0x00,
    TSL2591_GAIN_MED = 0x10,
    TSL2591_GAIN_HIGH = 0x20,
    TSL2591_GAIN_MAX = 0x30
};

typedef struct _tsl2591_t {
    int32_t fd;
    uint8_t int_time;
    uint8_t gain;
} tsl2591_t;

tsl2591_t *tsl2591_create(uint8_t dev_id);
double tsl2591_getLux(tsl2591_t *tsl);
void tsl2591_free(tsl2591_t *tsl);

#endif
