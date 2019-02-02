#ifndef BMP280_SENSOR_H
#define BMP280_SENSOR_H

#include <stdint.h>
#include "BMP280/bmp280.h"

typedef struct _bmp {
    struct bmp280_dev bmp;
} bmp280_t;

bmp280_t *bmp280_create(uint8_t dev_id);
double bmp280_getTemp(bmp280_t *bmp);
double bmp280_getPress(bmp280_t *bmp);
void bmp280_free(bmp280_t *bmp);

#endif
