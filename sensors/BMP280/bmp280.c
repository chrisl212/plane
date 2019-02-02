#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "bmp280.h"

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

bmp280_t *bmp280_create(uint8_t dev_id) {
    bmp280_t *bmp_p;
    struct bmp280_dev bmp;    
    struct bmp280_config conf;
    int8_t rslt;

    bmp_p = malloc(sizeof(*bmp_p));

    bmp.dev_id = dev_id;
    bmp.intf = BMP280_I2C_INTF;
    bmp.read = i2c_rx;
    bmp.write = i2c_tx;
    bmp.delay_ms = delay;

    rslt = bmp280_init(&bmp);
    printf("BMP280 PID: 0x%x\n", bmp.chip_id);
    
    rslt = bmp280_get_config(&conf, &bmp);
    conf.filter = BMP280_FILTER_COEFF_16;
    conf.os_pres = BMP280_OS_16X;
    conf.os_temp = BMP280_OS_2X;
    conf.odr = BMP280_ODR_62_5_MS;
    rslt = bmp280_set_config(&conf, &bmp);
    rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
    
    bmp_p->bmp = bmp;

    return bmp_p;
}

double bmp280_getTemp(bmp280_t *bmp) {
    struct bmp280_uncomp_data ucomp_data;
    int32_t temp32, rslt;

    rslt = bmp280_get_uncomp_data(&ucomp_data, &(bmp->bmp));

    temp32 = bmp280_comp_temp_32bit(ucomp_data.uncomp_temp, &(bmp->bmp));
    return bmp280_comp_temp_double(ucomp_data.uncomp_temp, &(bmp->bmp));
}

double bmp280_getPress(bmp280_t *bmp) {
    struct bmp280_uncomp_data ucomp_data;
    int32_t rslt;
    uint32_t pres32, pres64;

    rslt = bmp280_get_uncomp_data(&ucomp_data, &(bmp->bmp));

    pres32 = bmp280_comp_pres_32bit(ucomp_data.uncomp_press, &(bmp->bmp));
    pres64 = bmp280_comp_pres_64bit(ucomp_data.uncomp_press, &(bmp->bmp));
    return bmp280_comp_pres_double(ucomp_data.uncomp_press, &(bmp->bmp));
}

void bmp280_free(bmp280_t *bmp) {
    free(bmp);
}
