#ifndef LSM9DS0_H
#define LSM9DS0_H

#include <stdint.h>

typedef struct _coord_t {
    double x;
    double y;
    double z;
} coord_t;

typedef struct _lsm9ds0_t {
    int32_t gyro_fd;
    int32_t accel_fd;    
} lsm9ds0_t;

lsm9ds0_t *lsm9ds0_create(uint8_t gyro_id, uint8_t accel_id);
coord_t lsm9ds0_getAccel(lsm9ds0_t *lsm);
void lsm9ds0_free(lsm9ds0_t *lsm);

#endif
