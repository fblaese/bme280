CC = gcc
CFLAGS = -lwiringPi -lm

bme280: bme280.c bme280.h
	$(CC) $(CFLAGS) -o $@ $<
