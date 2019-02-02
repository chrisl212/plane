#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <math.h>
#include "sensors/BMP280/bmp280.h"
#include "sensors/TSL2591/tsl2591.h"
#include "sensors/LSM9DS0/lsm9ds0.h"

#define FPGA_ADDR 0x3C
#define PRESS_ADDR 0x77
#define LUX_ADDR 0x29
#define GYRO_ADDR 0x6B
#define ACCEL_ADDR 0x1D

void print_b(uint16_t x) {
    for (int i = 15; i >= 0; i--) 
        printf("%d", ((1 << i) & x) ? 1 : 0);
    printf("\n");
}

int main(int argc, char **argv) {
    bmp280_t *bmp;
    tsl2591_t *tsl;
    lsm9ds0_t *lsm;
    double press, temp, lux;
    int32_t fpga;
    coord_t c;
    double x, y, z;

    wiringPiSetup();

    bmp = bmp280_create(PRESS_ADDR);
    tsl = tsl2591_create(LUX_ADDR);
    lsm = lsm9ds0_create(GYRO_ADDR, ACCEL_ADDR);
    fpga = wiringPiI2CSetup(FPGA_ADDR);

    for (;;) {
        for (int i = 0; i < 100; i++) {
            //BMP280
            double temp_new = bmp280_getTemp(bmp);
            double press_new = bmp280_getPress(bmp);

            //tsl2591_t
            double lux_new = tsl2591_getLux(tsl);

            //LSM9DS0
            c = lsm9ds0_getAccel(lsm);

            if (temp_new != temp || press_new != press || lux != lux_new) {
                press = press_new;
                temp = temp_new;
                lux = lux_new;
                printf("\rTemperature: %.2lf F, Pressure: %.2lf hPa, Lux: %.2lf L, x: %.3f, y: %.3f, z: %.3f", temp*9.0/5.0 + 32, press/100.0, lux, c.x, c.y, c.z);
                fflush(stdout);
            }
           
            x = fabs(c.x);
            y = fabs(c.y);
            z = fabs(c.z); 
            //LEDs
            wiringPiI2CWriteReg8(fpga, 0x00, (uint8_t)(x*100.0/(x+y+z)));
            wiringPiI2CWriteReg8(fpga, 0x01, (uint8_t)(y*100.0/(x+y+z)));
            wiringPiI2CWriteReg8(fpga, 0x02, (uint8_t)(z*100.0/(x+y+z)));

            delay(10);
        }
    }


    return 0;
}
