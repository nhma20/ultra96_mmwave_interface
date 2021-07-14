# Ultra96V2 FPGA mmWave interface
Interfacing with mmWave-device (IWR6843AOP) from Ultra96V2 FPGA

### Prerequisites
Tested with:
- Ubuntu 20.04.2 LTS (host PC)
- Vivado / Vitis 2021.1 (with cable drivers and board files installed)
Board files installed by creating new project, and pressing refresh (lower left corner) when looking for boards. Ultra96V2 should appear now. Can close Vivado before finishing creation of new project. 



### Ultra96 FPGA JTAG programming
1. Follow instructions at https://www.hackster.io/BryanF/ultra96-v2-vivado-2020-2-basic-hardware-platform-6b32b8 to create basic hardware platform.
2. Then follow https://www.hackster.io/BryanF/ultra96-v2-vitis-2020-2-hello-world-from-arm-a53-2d952a to use hardware platform to create simple program to configure PS (needed to get clock to PL).
3. From Vitis, run simple application as debug to initialize PS - and keep it running.
![Alt text](https://github.com/nhma20/ultra96_mmwave_interface/blob/main/Pictures/Screenshotfrom2021-07-1318-04-27.png?raw=true)
5. From Vivado, program device with .bit-file to initialize FPGA with custom design, while PS clock is routed to PL. (https://forums.xilinx.com/t5/Xilinx-Evaluation-Boards/how-to-use-PS-Clock-for-PL-logic/m-p/783506/highlight/true#M15628)
6. FPGA can be re-programmed as usual now, and the PL clock will function as long as Vitis debug keeps PS configured.


### Prebuilt hardware platform 
1. Download .zip folder
```sh
cd ~/Downloads/ && wget -O ultra96_hw_platform.zip https://nextcloud.sdu.dk/index.php/s/NtNCWbADrXACTwf/download
```
2. Extract .zip to Vivado workspace (can be anywhere)
```sh
cd ~/Downloads/ && sudo unzip ~/Downloads/ultra96_hw_platform.zip -d /tools/Xilinx/Vivado/2021.1/bin/
```
3. Open Vivado
```
cd /tools/Xilinx/Vivado/2021.1/bin/ && sudo ./vivado
```
4. Click 'Open project' and open 'ultra96_hw_platform'
5. (new terminal) Open Vitis
```
cd /tools/Xilinx/Vitis/2021.1/bin/ && sudo ./vitis
```
6. When prompted for a workspace, navigate to /tools/Xilinx/Vivado/2021.1/bin/ultra96_hw_platform and launch


### MISC

### TODO
1. :green_circle: Configure ultra96 board, simple I/O program
2. :green_circle: Figure out how to use PS/PL clock without making new platform each time.
3. :yellow_circle: Find suitable interface on IWR6843AOP board
5. :yellow_circle: How to send config file from FPGA to IWR6843AOP
6. :yellow_circle: How to request/receive data from IWR6843AOP

