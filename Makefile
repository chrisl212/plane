all:
	gcc main.c sensors/LSM9DS0/lsm9ds0.c sensors/TSL2591/tsl2591.c sensors/BMP280/BMP280/bmp280.c sensors/BMP280/bmp280.c -lwiringPi -lm -o main.o
