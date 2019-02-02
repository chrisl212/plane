#include <stdlib.h>
#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "tsl2591.h"

tsl2591_t *tsl2591_create(uint8_t dev_id) {
    tsl2591_t *tsl;
    int32_t fd;
    uint8_t pid;

    tsl = malloc(sizeof(*tsl));
    fd = wiringPiI2CSetup(dev_id);
    pid = wiringPiI2CReadReg8(fd, 0xB2);
    printf("TSL2591 PID: 0x%x\n", pid);
    wiringPiI2CWriteReg8(fd, 0xA0, 0x03);
    wiringPiI2CWriteReg8(fd, 0xA1, 0x23); //high gain, 400 ms int
    
    tsl->fd = fd;
    tsl->int_time = TSL2591_INTEGRATIONTIME_400MS;
    tsl->gain = TSL2591_GAIN_HIGH;
    
    return tsl;
}

double tsl2591_getLux(tsl2591_t *tsl) {
    double gain_factor, int_factor, cpl, lux;
    uint16_t c0, c1;

    switch (tsl->int_time) {
        case (TSL2591_INTEGRATIONTIME_100MS): int_factor = 100.0;
                                              break;
        case (TSL2591_INTEGRATIONTIME_200MS): int_factor = 200.0;
                                              break;
        case (TSL2591_INTEGRATIONTIME_300MS): int_factor = 300.0;
                                              break;
        case (TSL2591_INTEGRATIONTIME_400MS): int_factor = 400.0;
                                              break;
        case (TSL2591_INTEGRATIONTIME_500MS): int_factor = 500.0;
                                              break;
        case (TSL2591_INTEGRATIONTIME_600MS): int_factor = 600.0;
                                              break;
    }

    switch (tsl->gain) {
        case (TSL2591_GAIN_LOW): gain_factor = 1.0;
                                 break;
        case (TSL2591_GAIN_MED): gain_factor = 25.0;
                                 break;
        case (TSL2591_GAIN_HIGH): gain_factor = 428.0;
                                 break;
        case (TSL2591_GAIN_MAX): gain_factor = 9876.0;
                                 break;
    }

    c0 = wiringPiI2CReadReg16(tsl->fd, 0xB4);
    c1 = wiringPiI2CReadReg16(tsl->fd, 0xB6);
    cpl = (gain_factor * int_factor) / TSL2591_LUX_DF;
    return ((double)c0 - (double)c1) * (1.0 - ((double)c1/(double)c0)) / cpl; 
}

void tsl2591_free(tsl2591_t *tsl) {
    free(tsl);
}
