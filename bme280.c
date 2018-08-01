#include "bme280.h"
#include <stdio.h>
#include <stdlib.h>
#include <wiringPiI2C.h>
#include <math.h>

static int read_calibration(int bme280, struct bme280_calibration *calibration) {
	calibration->dig_T1 = wiringPiI2CReadReg16(bme280, 0x88);
	calibration->dig_T2 = wiringPiI2CReadReg16(bme280, 0x8a);
	calibration->dig_T3 = wiringPiI2CReadReg16(bme280, 0x8c);

	calibration->dig_P1 = wiringPiI2CReadReg16(bme280, 0x8e);
	calibration->dig_P2 = wiringPiI2CReadReg16(bme280, 0x90);
	calibration->dig_P3 = wiringPiI2CReadReg16(bme280, 0x92);
	calibration->dig_P4 = wiringPiI2CReadReg16(bme280, 0x94);
	calibration->dig_P5 = wiringPiI2CReadReg16(bme280, 0x96);
	calibration->dig_P6 = wiringPiI2CReadReg16(bme280, 0x98);
	calibration->dig_P7 = wiringPiI2CReadReg16(bme280, 0x9a);
	calibration->dig_P8 = wiringPiI2CReadReg16(bme280, 0x9c);
	calibration->dig_P9 = wiringPiI2CReadReg16(bme280, 0x9e);

	calibration->dig_H1 = wiringPiI2CReadReg8(bme280, 0xa1);
	calibration->dig_H2 = wiringPiI2CReadReg16(bme280, 0xe1);
	calibration->dig_H3 = wiringPiI2CReadReg8(bme280, 0xe3);

	uint8_t reg_E5 = wiringPiI2CReadReg8(bme280, 0xe5);
	calibration->dig_H4 = (wiringPiI2CReadReg8(bme280, 0xe4) << 4) | (reg_E5 & 0xF);
	calibration->dig_H5 = (reg_E5 >> 4) | (wiringPiI2CReadReg8(bme280, 0xe6) << 4);

	calibration->dig_H6 = wiringPiI2CReadReg8(bme280, 0xe7);
}

static int32_t t_fine;
static int32_t compensate_t(int32_t adc_t, struct bme280_calibration *calib) {
	int32_t var1, var2, t;

	var1 = ((((adc_t >> 3) - ((int32_t) calib->dig_T1 << 1))) * ((int32_t) calib->dig_T2)) >> 11;
	var2 = (((((adc_t >> 4) - ((int32_t) calib->dig_T1)) * ((adc_t >> 4) - ((int32_t) calib->dig_T1))) >> 12) *
		((int32_t) calib->dig_T3)) >> 14;

	t_fine = var1 + var2;
	t = (t_fine * 5 + 128) >> 8;
	
	return t;
}

static uint32_t compensate_p(int32_t adc_p, struct bme280_calibration *calib) {
	int64_t var1, var2, p;

	var1 = ((int64_t) t_fine) - 128000;
	var2 = var1 * var1 * (int64_t) calib->dig_P6;
	var2 = var2 + ((var1 * (int64_t) calib->dig_P5) << 17);
	var2 = var2 + (((int64_t) calib->dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t) calib->dig_P3) >> 8) + ((var1 * (int64_t) calib->dig_P2) << 12);
	var1 = (((((int64_t) 1) << 47) + var1)) * ((int64_t) calib->dig_P1) >> 33;

	if (var1 == 0) {
		// avoid exception caused by division by zero
		return 0;
	}

	p = 1048576 - adc_p;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t) calib->dig_P9) * (p >> 13) * (p >> 13)) >> 25; var2 = (((int64_t) calib->dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t) calib->dig_P7) << 4);
	
	return p;
}

static uint32_t compensate_h(int32_t adc_h, struct bme280_calibration *calib) {
	int32_t v_x1_u32r;

	v_x1_u32r = (t_fine - ((int32_t) 76800));
	v_x1_u32r = (((((adc_h << 14) - (((int32_t) calib->dig_H4) << 20) - (((int32_t) calib->dig_H5) * v_x1_u32r)) +
		((int32_t) 16384)) >> 15) * (((((((v_x1_u32r * ((int32_t) calib->dig_H6)) >> 10) * (((v_x1_u32r *
		((int32_t) calib->dig_H3)) >> 11) + ((int32_t) 32768))) >> 10) + ((int32_t) 2097152)) *
		((int32_t) calib->dig_H2) + 8192) >> 14));

	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t) calib->dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

	return (v_x1_u32r >> 12);
}

int main(void) {
	int bme280 = wiringPiI2CSetup(bme280_address);
	if (bme280 == -1) {
		perror("wiringPiI2CSetup");
		exit(EXIT_FAILURE);
	}

	uint8_t bme280_id = wiringPiI2CReadReg8(bme280, 0xd0);
	if (bme280_id != 0x60) {
		fprintf(stderr, "Invalid sensor id!\n");
		exit(EXIT_FAILURE);
	}

	struct bme280_calibration calibration;
	read_calibration(bme280, &calibration);

	// set oversampling values
	uint8_t osrs_p = 1;
	uint8_t osrs_t = 1;
	uint8_t osrs_h = 1;
	wiringPiI2CWriteReg8(bme280, 0xf2, wiringPiI2CReadReg8(bme280, 0xf2) | osrs_h );
	wiringPiI2CWriteReg8(bme280, 0xf4, (osrs_p << 5) | (osrs_p << 2) | (wiringPiI2CReadReg8(bme280, 0xf4) & 0x3));

	// set IIR filter
	wiringPiI2CWriteReg8(bme280, 0xf5, (wiringPiI2CReadReg8(bme280, 0xf5) & 0xe1) | (0x0 << 1));

	// start measurement (set mode to forced)
	wiringPiI2CWriteReg8(bme280, 0xf4, (wiringPiI2CReadReg8(bme280, 0xf4) & 0xfc) | 0x1);

	while (wiringPiI2CReadReg8(bme280, 0xf3) & 0x8 != 0) {
	}

	// read measurements
	int32_t adc_t = wiringPiI2CReadReg8(bme280, 0xfa) << 12 | wiringPiI2CReadReg8(bme280, 0xfb) << 4 | wiringPiI2CReadReg8(bme280, 0xfc) & 0xf;
	uint32_t adc_p = wiringPiI2CReadReg8(bme280, 0xf7) << 12 | wiringPiI2CReadReg8(bme280, 0xf8) << 4 | wiringPiI2CReadReg8(bme280, 0xf9) & 0xf;
	uint32_t adc_h = wiringPiI2CReadReg8(bme280, 0xfd) << 8 | wiringPiI2CReadReg8(bme280, 0xfe);

	int32_t temp = compensate_t(adc_t, &calibration);
	uint32_t pres = compensate_p(adc_p, &calibration);
	uint32_t hum = compensate_h(adc_h, &calibration);

	double temp_d = (double) temp / 100;
	double pres_d = (double) pres / 256 / 100;
	double hum_d = (double) hum / 1024;

	// normaldruck
	int height = 309;
	pres_d = pres_d / pow(1 - 0.0065 * height / ((temp_d + 0.0065 * height) + 273.15), 0.03416 / 0.0065);

	printf("Temperature: %.2f °C\n", temp_d);
	printf("Pressure: %.2f hPa\n", pres_d);
	printf("Humidity: %.2f %\n", hum_d);
}
