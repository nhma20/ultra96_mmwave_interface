# Ultra96V2 FPGA mmWave interface
Interfacing with mmWave-device (IWR6843AOP) from Ultra96V2 FPGA

### Prerequisites
Tested with:
- Ubuntu 20.04.2 LTS (host PC)
- Vivado / Vitis 2021.1



### Ultra96 FPGA JTAG programming
1. Follow instructions at https://www.hackster.io/BryanF/ultra96-v2-vivado-2020-2-basic-hardware-platform-6b32b8 to create basic hardware platform.
2. Then follow https://www.hackster.io/BryanF/ultra96-v2-vitis-2020-2-hello-world-from-arm-a53-2d952a to use hardware platform to create simple program to configure PS (needed to get clock to PL).
3. From Vitis, run simple application as debug to initialize PS - and keep it running.
![Alt text](https://github.com/nhma20/ultra96_mmwave_interface/blob/main/Pictures/Screenshotfrom2021-07-1318-04-27.png?raw=true)
5. From Vivado, program device with .bit-file to initialize FPGA with custom design, while PS clock is routed to PL. (https://forums.xilinx.com/t5/Xilinx-Evaluation-Boards/how-to-use-PS-Clock-for-PL-logic/m-p/783506/highlight/true#M15628)
6. FPGA can be re-programmed and tested with a working clock as long as Vitis debug keeps PS configured.


### MISC

### TODO
1. :yellow_circle:

