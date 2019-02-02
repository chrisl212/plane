#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "BMP280/bmp280.h"

#define FPGA_ADDR 0x3C
#define PRESS_ADDR 0x77
#define LUX_ADDR 0x29
#define GYRO_ADDR 0x6B
#define ACCEL_ADDR 0x1D

int8_t i2c_tx(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    int dev = wiringPiI2CSetup(dev_id);
    for (int i = 0; i < len; i++) {
        wiringPiI2CWriteReg8(dev, reg_addr+i, data[i]);
    }
    close(dev);
    return 0;
}

int8_t i2c_rx(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    int dev = wiringPiI2CSetup(dev_id);
    for (int i = 0; i < len; i++) {
        data[i] = wiringPiI2CReadReg8(dev, reg_addr+i);
    }
    close(dev);
    return 0;
}

void print_b(uint16_t x) {
    for (int i = 15; i >= 0; i--) 
        printf("%d", ((1 << i) & x) ? 1 : 0);
    printf("\n");
}

int main(int argc, char **argv) {
    struct bmp280_dev bmp;
    
    wiringPiSetup();

    int tsl = wiringPiI2CSetup(LUX_ADDR);
    int pid = wiringPiI2CReadReg8(tsl, 0xB2);
    printf("TSL2591 PID: 0x%x\n", pid);
    wiringPiI2CWriteReg8(tsl, 0xA0, 0x03);
    wiringPiI2CWriteReg8(tsl, 0xA1, 0x23); //high gain, 400 ms int

    int gyro = wiringPiI2CSetup(GYRO_ADDR);
    pid = wiringPiI2CReadReg8(gyro, 0x0F);
    printf("LSM9DS0 Gyro PID: 0x%x\n", pid);
    wiringPiI2CWriteReg8(gyro, 0x20, 0x0F);

    int accel = wiringPiI2CSetup(ACCEL_ADDR);
    pid = wiringPiI2CReadReg8(accel, 0x0F);
    printf("LSM9DS0 Accelerometer PID: 0x%x\n", pid);
    wiringPiI2CWriteReg8(accel, 0x20, 0x67);
    wiringPiI2CWriteReg8(accel, 0x21, 0x00);
    //wiringPiI2CWriteReg8(accel, 0x21, 0x18); //8g

    bmp.dev_id = PRESS_ADDR;
    bmp.intf = BMP280_I2C_INTF;
    bmp.read = i2c_rx;
    bmp.write = i2c_tx;
    bmp.delay_ms = delay;

    int rslt = bmp280_init(&bmp);
    printf("BMP280 PID: 0x%x\n\n", bmp.chip_id);
    
    struct bmp280_config conf;

    /* Always read the current settings before writing, especially when
     * all the configuration is not modified 
     */
    rslt = bmp280_get_config(&conf, &bmp);
    /* Check if rslt == BMP280_OK, if not, then handle accordingly */

    /* Overwrite the desired settings */
    conf.filter = BMP280_FILTER_COEFF_16;
    conf.os_pres = BMP280_OS_16X;
    conf.os_temp = BMP280_OS_2X;
    conf.odr = BMP280_ODR_62_5_MS;

    rslt = bmp280_set_config(&conf, &bmp);
    /* Check if rslt == BMP280_OK, if not, then handle accordingly */

    /* Always set the power mode after setting the configuration */
    rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);

    double press, temp, lux;
    int fpga = wiringPiI2CSetup(FPGA_ADDR);
    for (;;) {
        for (int i = 0; i < 100; i++) {

            //BMP280
            struct bmp280_uncomp_data ucomp_data;
            uint8_t meas_dur = bmp280_compute_meas_time(&bmp);

            rslt = bmp280_get_uncomp_data(&ucomp_data, &bmp);
            /* Check if rslt == BMP280_OK, if not, then handle accordingly */

            int32_t temp32 = bmp280_comp_temp_32bit(ucomp_data.uncomp_temp, &bmp);
            uint32_t pres32 = bmp280_comp_pres_32bit(ucomp_data.uncomp_press, &bmp);
            uint32_t pres64 = bmp280_comp_pres_64bit(ucomp_data.uncomp_press, &bmp);
            double temp_new = bmp280_comp_temp_double(ucomp_data.uncomp_temp, &bmp);
            double press_new = bmp280_comp_pres_double(ucomp_data.uncomp_press, &bmp);


            //TSL2591
            uint16_t c0 = wiringPiI2CReadReg16(tsl, 0xB4);
            uint16_t c1 = wiringPiI2CReadReg16(tsl, 0xB6);
            double cpl = (428.0 * 400.0) / 408.0;
            double lux_new = ((double)c0 - (double)c1) * (1.0 - ((double)c1/(double)c0)) / cpl; 

            //LSM9DS0
            uint16_t xl = wiringPiI2CReadReg8(accel, 0x28);
            uint16_t xh = wiringPiI2CReadReg8(accel, 0x29);
            int16_t xi = (int16_t)(xl | (xh << 8));
            uint16_t yl = wiringPiI2CReadReg8(accel, 0x2A);
            uint16_t yh = wiringPiI2CReadReg8(accel, 0x2B);
            int16_t yi = (int16_t)(yl | (yh << 8));
            uint16_t zl = wiringPiI2CReadReg8(accel, 0x2C);
            uint16_t zh = wiringPiI2CReadReg8(accel, 0x2D);
            int16_t zi = (int16_t)(zl | (zh << 8));
            double x = xi*0.061/1000.0;
            double y = yi*0.061/1000.0;
            double z = zi*0.061/1000.0;

            if (temp_new != temp || press_new != press || lux != lux_new) {
                press = press_new;
                temp = temp_new;
                lux = lux_new;
                printf("\rTemperature: %.2lf F, Pressure: %.2lf hPa, Lux: %.2lf L, x: %.3f, y: %.3f, z: %.3f", temp*9.0/5.0 + 32, press/100.0, lux, x*0.244, y, z);
                fflush(stdout);
            }
            
            //LEDs
            x = abs(xi);
            y = abs(yi);
            z = abs(zi);
            wiringPiI2CWriteReg8(fpga, 0x00, x*100.0/(x+y+z));
            wiringPiI2CWriteReg8(fpga, 0x01, y*100.0/(x+y+z));
            wiringPiI2CWriteReg8(fpga, 0x02, z*100.0/(x+y+z));

            bmp.delay_ms(meas_dur); /* Measurement time */
        }
    }


    return 0;
}
