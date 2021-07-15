# Ultra96V2 FPGA mmWave interface
Interfacing with mmWave-device (IWR6843AOPEVM) from Ultra96V2 FPGA

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
1. Download hardware platform and vivado project .zip folders
```sh
cd ~/Downloads/ && wget -O ultra96_vivado_proj.zip https://nextcloud.sdu.dk/index.php/s/DiZxBmsK6Q5azkL/download && wget -O ultra96_hw_platform.zip https://nextcloud.sdu.dk/index.php/s/qwARBD2TFrrTEJ2/download
```
2. Extract .zip folders to workspaces (can be anywhere, here home is chosen)
```sh
cd ~/Downloads/ && unzip ~/Downloads/ultra96_hw_platform.zip -d ~ && unzip ~/Downloads/ultra96_vivado_proj.zip -d ~
```
3. Open Vivado
```
cd /tools/Xilinx/Vivado/2021.1/bin/ && sudo ./vivado
```
4. Click 'Open project' and open 'ultra96_vivado_prj' from home installation folder
5. (new terminal) Open Vitis
```
cd /tools/Xilinx/Vitis/2021.1/bin/ && sudo ./vitis
```
6. When prompted for a workspace, navigate to ~/ultra96_hw_platform and launch
7. In Vitis, left click 'ultra96_simple_application_system' in the explorer window and 'Debug as -> Launch hardware'. This initializes PS side. 
8. In Vivado, Program device with bitstream and all should work as expected.
9. Modify Vivado project as needed, generate new bitstream, and program device to test added functionality.


### MISC

### TODO
1. :green_circle: Configure ultra96 board, simple I/O program
2. :green_circle: Figure out how to use PS/PL clock without making new platform each time.
3. :yellow_circle: Find suitable interface on IWR6843AOPEVM board
5. :yellow_circle: How to send config file from FPGA to IWR6843AOPEVM
6. :yellow_circle: How to request/receive data from IWR6843AOPEVM

