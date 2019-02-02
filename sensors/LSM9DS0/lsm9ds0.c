#include <stdlib.h>
#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "lsm9ds0.h"

lsm9ds0_t *lsm9ds0_create(uint8_t gyro_id, uint8_t accel_id) {
    lsm9ds0_t *lsm;
    int32_t gyro_fd, accel_fd;
    uint8_t pid;

    lsm = malloc(sizeof(*lsm));

    gyro_fd = wiringPiI2CSetup(gyro_id);
    pid = wiringPiI2CReadReg8(gyro_fd, 0x0F);
    printf("LSM9DS0 Gyro PID: 0x%x\n", pid);
    wiringPiI2CWriteReg8(gyro_fd, 0x20, 0x0F);

    accel_fd = wiringPiI2CSetup(accel_id);
    pid = wiringPiI2CReadReg8(accel_fd, 0x0F);
    printf("LSM9DS0 Accelerometer PID: 0x%x\n", pid);
    wiringPiI2CWriteReg8(accel_fd, 0x20, 0x67);
    wiringPiI2CWriteReg8(accel_fd, 0x21, 0x18); //8g

    lsm->accel_fd = accel_fd;
    lsm->gyro_fd = gyro_fd;
    return lsm; 
}

coord_t lsm9ds0_getAccel(lsm9ds0_t *lsm) {
    uint16_t xl, xh, yl, yh, zl, zh;
    int16_t xi, yi, zi;
    coord_t c;

    xl = wiringPiI2CReadReg8(lsm->accel_fd, 0x28);
    xh = wiringPiI2CReadReg8(lsm->accel_fd, 0x29);
    xi = (int16_t)(xl | (xh << 8));
    
    yl = wiringPiI2CReadReg8(lsm->accel_fd, 0x2A);
    yh = wiringPiI2CReadReg8(lsm->accel_fd, 0x2B);
    yi = (int16_t)(yl | (yh << 8));
    
    zl = wiringPiI2CReadReg8(lsm->accel_fd, 0x2C);
    zh = wiringPiI2CReadReg8(lsm->accel_fd, 0x2D);
    zi = (int16_t)(zl | (zh << 8));
    c.x = xi*0.244/1000.0;
    c.y = yi*0.244/1000.0;
    c.z = zi*0.244/1000.0;
    return c;
}

void lsm9ds0_free(lsm9ds0_t *lsm) {
    free(lsm);
}
