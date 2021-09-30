/**********************************************************************************************************
 * Includes:
 **********************************************************************************************************/
#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"
#include "stdlib.h"
#include "stdbool.h"
#include "sleep.h"

#include "xbram.h"
#include "xgpio.h"
#include "xtime_l.h"

/**********************************************************************************************************
 * Compilation:
 **********************************************************************************************************/

//#define DEBUG

#define CONSTANT_SAMPLING \
					0
#define STEPWISE_SAMPLING \
					1

#define SAMPLING 	CONSTANT_SAMPLING

/**********************************************************************************************************
 * Defines:
 **********************************************************************************************************/

#define BRAM(A)     ((volatile u32*)px_config->MemBaseAddress)[A]
#define BRAM_S(A)   ((volatile s32*)px_config->MemBaseAddress)[A]
#define BRAM_FP(A)  ((volatile float*)px_config->MemBaseAddress)[A]

#define MAG_F_S_HZ	1000
#define MMW_F_S_HZ	30

#define N_MAG_SAMPLES	1000
#define	N_MMWAVE_SAMPLES \
						(int)(N_MAG_SAMPLES / ((int)(MAG_F_S_HZ/MMW_F_S_HZ)+1))

/**********************************************************************************************************
 * Typedefs:
 **********************************************************************************************************/

typedef struct
{
	float			x;
	float			y;
	float			z;
} mmWaveSample_t;

typedef struct
{
	mmWaveSample_t	px_readings[8];
} mmWaveReading_t;

typedef struct
{
	uint16_t		x;
	uint16_t		y;
	uint16_t		z;
} MagSample_t;

typedef struct
{
	uint64_t		ul_time;
	MagSample_t 	px_reading[4];
} MagReading_t;

typedef struct
{
	double			gyro_x;
	double			gyro_y;
	double			gyro_z;
	double			acc_x;
	double			acc_y;
	double			acc_z;
//	int16_t			gyro_x;
//	int16_t			gyro_y;
//	int16_t			gyro_z;
//	int16_t			acc_x;
//	int16_t			acc_y;
//	int16_t			acc_z;
} IMUReading_t;

/**********************************************************************************************************
 * Forward declarations:
 **********************************************************************************************************/

XGpio 				x_gpio;

XBram           	x_bram;
XBram_Config    	*px_config;

uint8_t 			ucAXIInit();

MagReading_t		XGetMagReading();
mmWaveReading_t		XGetmmWaveReading();
IMUReading_t		XGetIMUReading();

double 				dAccConvert(int16_t s_acc, double d_acc_range);
double 				dGyroConvert(int16_t s_gyro, double d_gyro_range);

void				vPrintReadings(MagReading_t *px_mag_data, mmWaveReading_t *px_mmWave_data);
void				vPrintReading(MagReading_t x_mag_reading, mmWaveReading_t x_mmw_reading, IMUReading_t x_imu_reading);

void				vStepwiseSampling();
void				vConstantSampling();

/**********************************************************************************************************
 * Methods:
 **********************************************************************************************************/

int main()
{
    init_platform();

	ucAXIInit();

#ifdef DEBUG
	xil_printf("Ready\n\r");
#endif

	if (SAMPLING == CONSTANT_SAMPLING)
	{
		vConstantSampling();
	}
	else
	{
		vStepwiseSampling();
	}
}

uint8_t 	ucAXIInit()
{
	// GPIO Initialization
#ifdef DEBUG
	xil_printf("Initializing GPIO...\n\r");
#endif
	int x_status 	= 	XGpio_Initialize(&x_gpio, XPAR_GPIO_0_DEVICE_ID);

	if (x_status != XST_SUCCESS)
	{
		return 0;
	}

	XGpio_SetDataDirection(&x_gpio, 1, 0xFFFFFFFF);

#ifdef DEBUG
	xil_printf("Done.\r\n");
#endif

	// BRAM initialization
#ifdef DEBUG
	xil_printf("Initializing BRAM...\n\r");
#endif

	px_config = XBram_LookupConfig(XPAR_BRAM_0_DEVICE_ID);
	if (px_config == (XBram_Config *) NULL) {
		return XST_FAILURE;
	}

	x_status 	= 	XBram_CfgInitialize(&x_bram, px_config,
			px_config->CtrlBaseAddress);

	if (x_status != XST_SUCCESS) {
		return 0;
	}

#ifdef DEBUG
	xil_printf("Done.\r\n");
#endif

	return 1;
}

MagReading_t	XGetMagReading()
{
	MagReading_t		x_reading;

	uint64_t			ul_time		=	0;
	XTime_GetTime(&ul_time);

	x_reading.ul_time 				=	ul_time;

	x_reading.px_reading[0].x		= 	BRAM(0);
	x_reading.px_reading[0].y 		= 	BRAM(1);
	x_reading.px_reading[0].z 		= 	BRAM(2);

	x_reading.px_reading[1].x		= 	BRAM(3);
	x_reading.px_reading[1].y 		= 	BRAM(4);
	x_reading.px_reading[1].z 		= 	BRAM(5);

	x_reading.px_reading[2].x		= 	BRAM(6);
	x_reading.px_reading[2].y 		= 	BRAM(7);
	x_reading.px_reading[2].z 		= 	BRAM(8);

	x_reading.px_reading[3].x		= 	BRAM(9);
	x_reading.px_reading[3].y 		= 	BRAM(10);
	x_reading.px_reading[3].z 		= 	BRAM(11);

	return x_reading;
}

mmWaveReading_t		XGetmmWaveReading()
{
	mmWaveReading_t	x_reading;

	x_reading.px_readings[0].x		=	BRAM_FP(12);
	x_reading.px_readings[0].y		=	BRAM_FP(13);
	x_reading.px_readings[0].z		=	BRAM_FP(14);
	x_reading.px_readings[1].x		=	BRAM_FP(15);
	x_reading.px_readings[1].y		=	BRAM_FP(16);
	x_reading.px_readings[1].z		=	BRAM_FP(17);
	x_reading.px_readings[2].x		=	BRAM_FP(18);
	x_reading.px_readings[2].y		=	BRAM_FP(19);
	x_reading.px_readings[2].z		=	BRAM_FP(20);
	x_reading.px_readings[3].x		=	BRAM_FP(21);
	x_reading.px_readings[3].y		=	BRAM_FP(22);
	x_reading.px_readings[3].z		=	BRAM_FP(23);
	x_reading.px_readings[4].x		=	BRAM_FP(24);
	x_reading.px_readings[4].y		=	BRAM_FP(25);
	x_reading.px_readings[4].z		=	BRAM_FP(26);
	x_reading.px_readings[5].x		=	BRAM_FP(27);
	x_reading.px_readings[5].y		=	BRAM_FP(28);
	x_reading.px_readings[5].z		=	BRAM_FP(29);
	x_reading.px_readings[6].x		=	BRAM_FP(30);
	x_reading.px_readings[6].y		=	BRAM_FP(31);
	x_reading.px_readings[6].z		=	BRAM_FP(32);
	x_reading.px_readings[7].x		=	BRAM_FP(33);
	x_reading.px_readings[7].y		=	BRAM_FP(34);
	x_reading.px_readings[7].z		=	BRAM_FP(35);

	return x_reading;
}

IMUReading_t		XGetIMUReading()
{
	IMUReading_t	x_reading;

	x_reading.gyro_x				=	dGyroConvert(BRAM_S(36), 250);
	x_reading.gyro_y				=	dGyroConvert(BRAM_S(37), 250);
	x_reading.gyro_z				=	dGyroConvert(BRAM_S(38), 250);

	x_reading.acc_x					=	dAccConvert(BRAM_S(39), 9.81 * 2);
	x_reading.acc_y					=	dAccConvert(BRAM_S(40), 9.81 * 2);
	x_reading.acc_z					=	dAccConvert(BRAM_S(41), 9.81 * 2);

//	x_reading.gyro_x				=	BRAM_S(36);
//	x_reading.gyro_y				=	BRAM_S(37);
//	x_reading.gyro_z				=	BRAM_S(38);
//
//	x_reading.acc_x					=	BRAM_S(39);
//	x_reading.acc_y					=	BRAM_S(40);
//	x_reading.acc_z					=	BRAM_S(41);

//	x_reading.gyro_x				=	BRAM(36);
//	x_reading.gyro_y				=	BRAM(37);
//	x_reading.gyro_z				=	BRAM(38);
//
//	x_reading.acc_x					=	BRAM(39);
//	x_reading.acc_y					=	BRAM(40);
//	x_reading.acc_z					=	BRAM(41);

	return x_reading;
}

double 				dAccConvert(int16_t s_acc, double d_acc_range)
{
	return (double)((double) s_acc * (double) d_acc_range / 32768.);
}

double 				dGyroConvert(int16_t s_gyro, double d_gyro_range)
{
	return (double)((double) s_gyro * (double) d_gyro_range / 32768.) * 0.0174532925;
}

void				vPrintReadings(MagReading_t *px_mag_data, mmWaveReading_t *px_mmWave_data)
{
	int idx_mmwave = -1;
	char buffer[100];

	if (buffer == NULL)
	{
#ifdef DEBUG
		xil_printf("Couldn't allocate memory\n\r");
#endif
		while(1);
	}

	for (int i = 0; i < N_MAG_SAMPLES; i++)
	{
		if (i % ((int)((MAG_F_S_HZ / MMW_F_S_HZ) + 1)) == 0)
		{
			idx_mmwave++;
		}

		xil_printf("%u\t", px_mag_data[i].ul_time);

		for (int j = 0; j < 4; j++)
		{
			sprintf(buffer, "%u\t%u\t%u\t", px_mag_data[i].px_reading[j].x, px_mag_data[i].px_reading[j].y, px_mag_data[i].px_reading[j].z);
			xil_printf(buffer);
		}

		for (int j = 0; j < 8; j++)
		{
			sprintf(buffer, "%f\t%f\t%f", px_mmWave_data[idx_mmwave].px_readings[j].x, px_mmWave_data[idx_mmwave].px_readings[j].y, px_mmWave_data[idx_mmwave].px_readings[j].z);
//			sprintf(buffer, "%d\t%d\t%d", (int32_t)(1000*px_mmWave_data[idx_mmwave].px_readings[j].x), (int32_t)(1000*px_mmWave_data[idx_mmwave].px_readings[j].y), (int32_t)(1000*px_mmWave_data[idx_mmwave].px_readings[j].z));
			xil_printf(buffer);

			if (j == 7)
				xil_printf("\n");

			else
				xil_printf("\t");
		}

#ifdef DEBUG
		xil_printf("\r");
#endif
	}

	xil_printf("end\n");


#ifdef DEBUG
		xil_printf("\r");
		sleep(1);
#endif
}

void				vPrintReading(MagReading_t x_mag_reading, mmWaveReading_t x_mmw_reading, IMUReading_t x_imu_reading)
{
	char			buffer[1000];

//	xil_printf("%u\t", x_mag_reading.ul_time);
//
//	xil_printf("%u\t%u\t%u\t\t\t%u\t%u\t%u\t\t\t%u\t%u\t%u\t\t\t%u\t%u\t%u\t",
//			x_mag_reading.px_reading[0].x, x_mag_reading.px_reading[0].y, x_mag_reading.px_reading[0].z,
//			x_mag_reading.px_reading[1].x, x_mag_reading.px_reading[1].y, x_mag_reading.px_reading[1].z,
//			x_mag_reading.px_reading[2].x, x_mag_reading.px_reading[2].y, x_mag_reading.px_reading[2].z,
//			x_mag_reading.px_reading[3].x, x_mag_reading.px_reading[3].y, x_mag_reading.px_reading[3].z
//	);
//
	sprintf(buffer, "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t",
			x_mmw_reading.px_readings[0].x,
			x_mmw_reading.px_readings[0].y,
			x_mmw_reading.px_readings[0].z,
			x_mmw_reading.px_readings[1].x,
			x_mmw_reading.px_readings[1].y,
			x_mmw_reading.px_readings[1].z,
			x_mmw_reading.px_readings[2].x,
			x_mmw_reading.px_readings[2].y,
			x_mmw_reading.px_readings[2].z,
			x_mmw_reading.px_readings[3].x,
			x_mmw_reading.px_readings[3].y,
			x_mmw_reading.px_readings[3].z,
			x_mmw_reading.px_readings[4].x,
			x_mmw_reading.px_readings[4].y,
			x_mmw_reading.px_readings[4].z,
			x_mmw_reading.px_readings[5].x,
			x_mmw_reading.px_readings[5].y,
			x_mmw_reading.px_readings[5].z,
			x_mmw_reading.px_readings[6].x,
			x_mmw_reading.px_readings[6].y,
			x_mmw_reading.px_readings[6].z,
			x_mmw_reading.px_readings[7].x,
			x_mmw_reading.px_readings[7].y,
			x_mmw_reading.px_readings[7].z
	);

	xil_printf(buffer);

	sprintf(buffer, "\t%f\t%f\t%f\t\t%f\t%f\t%f\n\r",
			x_imu_reading.gyro_x,
			x_imu_reading.gyro_y,
			x_imu_reading.gyro_z,
			x_imu_reading.acc_x,
			x_imu_reading.acc_y,
			x_imu_reading.acc_z
	);

	xil_printf(buffer);
}

void				vStepwiseSampling()
{
#ifdef DEBUG
	xil_printf("Allocating sample memory\n\r");
#endif

	MagReading_t 	*px_mag_data 	= 	malloc(N_MAG_SAMPLES*sizeof(MagReading_t));
	mmWaveReading_t	*px_mmWave_data	=	malloc(N_MMWAVE_SAMPLES*sizeof(mmWaveReading_t));

	if (px_mag_data == NULL || px_mmWave_data == NULL)
	{
		xil_printf("Couldn't allocate memory\n\r");
		while(1);
	}

    while(1)
    {
#ifdef DEBUG
    	xil_printf("Beginning sampling\n\r");
#endif

    	int idx_mmw = 0;
    	for (int i = 0; i < N_MAG_SAMPLES; i++)
    	{
#ifdef DEBUG
    		xil_printf("At sample %u out of %u\n\r", i, N_MAG_SAMPLES);
#endif

    		uint32_t	ul_gpio_read		=	XGpio_DiscreteRead(&x_gpio, 1);
    		uint32_t	ul_gpio_last_read	=	XGpio_DiscreteRead(&x_gpio, 1);
#ifdef DEBUG
    		xil_printf("Waiting for rising edge\n\r");
#endif

    		while(!((ul_gpio_read & 0x00000001) && !(ul_gpio_last_read & 0x00000001)))
    		{
    			ul_gpio_last_read			=	ul_gpio_read;
    			ul_gpio_read				=	XGpio_DiscreteRead(&x_gpio, 1);
    		}
#ifdef DEBUG
    		xil_printf("Obtained rising edge\n\r");
    		xil_printf("Sampling magnetometer\n\r");
#endif

    		((MagReading_t *)px_mag_data)[i]	=	XGetMagReading();

    		if (i % ((int)((MAG_F_S_HZ / MMW_F_S_HZ) + 1)) == 0)
    		{
#ifdef DEBUG
    			xil_printf("Sampling mmWave\n\r");
#endif

    			((mmWaveReading_t *)px_mmWave_data)[idx_mmw++]	=	XGetmmWaveReading();
    		}

#ifdef DEBUG
    			xil_printf("Finished single sample\n\r");
#endif
    	}

#ifdef DEBUG
    			xil_printf("Finished sampling sequence\n\r");
#endif

    	vPrintReadings(px_mag_data, px_mmWave_data);
    }
}

void			vConstantSampling()
{
	mmWaveReading_t	x_mmw_reading;
	MagReading_t	x_mag_reading;
	IMUReading_t	x_imu_reading;

    while(1)
    {
		x_mmw_reading				=	XGetmmWaveReading();
    	x_mag_reading				=	XGetMagReading();
    	x_imu_reading				=	XGetIMUReading();

    	vPrintReading(x_mag_reading, x_mmw_reading, x_imu_reading);
    }
}
