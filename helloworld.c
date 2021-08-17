/******************************************************************************
*
* Copyright (C) 2009 - 2014 Xilinx, Inc.  All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* Use of the Software is limited solely to applications:
* (a) running on a Xilinx device, or
* (b) that interact with a Xilinx device through a bus or interconnect.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* XILINX  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of the Xilinx shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from Xilinx.
*
******************************************************************************/

/*
 * helloworld.c: simple test application
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"
#include "xbram.h"



#define BRAM(A)        ((volatile u32*)px_config->MemBaseAddress)[A]

XBram             x_bram;
XBram_Config    *px_config;


void            vInitBram();


int main()
{
    init_platform();

    vInitBram();

    print("Hello World\n\r");
    print("Successfully ran Hello World application");

    uint32_t read_data;
    uint32_t data_array[32];
    while (1){

    	usleep(200000);
    	//printf("\e[1;1H\e[2J \n\r Points (x values only):");
    	//printf("Points (x values only):");
    	for(int i = 0; i < 32; i++){
        	read_data = (uint32_t)BRAM(i); // 0 = BRAM address to read from
        	data_array[i] = read_data;
        	//printf("\n\r  %u", read_data);
    	}

    	print("\n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n\r");
    	printf("Points (x values only): \n\r  %u \n\r  %u \n\r  %u \n\r  %u \n\r  %u \n\r  %u \n\r  %u \n\r  %u \n\r  %u \n\r  %u \n\r  %u \n\r  %u \n\r  %u \n\r  %u \n\r  %u \n\r  %u \n\r  %u \n\r  %u \n\r  %u \n\r  %u \n\r  %u \n\r  %u \n\r  %u \n\r  %u \n\r  %u \n\r  %u \n\r  %u \n\r  %u \n\r  %u \n\r  %u \n\r  %u \n\r  %u \n \n \n \n \n \n \n", data_array[0],data_array[1],data_array[2],data_array[3],data_array[4], data_array[5], data_array[6], data_array[7],
				data_array[8],data_array[9],data_array[10],data_array[3],data_array[12],data_array[13], data_array[14], data_array[15], data_array[16], data_array[17], data_array[18], data_array[19],
				data_array[20], data_array[21], data_array[22], data_array[23],data_array[24], data_array[25],data_array[26], data_array[27],data_array[28], data_array[29], data_array[30], data_array[31]);


    }

    cleanup_platform();
    return 0;
}



void            vInitBram()
{
    px_config     =    XBram_LookupConfig(XPAR_BRAM_0_DEVICE_ID);
    XBram_CfgInitialize(&x_bram, px_config, px_config->CtrlBaseAddress);
}
