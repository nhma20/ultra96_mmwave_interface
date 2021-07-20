# Ultra96V2 FPGA mmWave interface
Interfacing with mmWave-device (IWR6843AOPEVM) from Ultra96V2 FPGA

### Prerequisites
Tested with:
- Ubuntu 20.04.2 LTS (host PC)
- Vivado / Vitis 2021.1 (with cable drivers and board files installed)
Board files installed by creating new project, and pressing refresh (lower left corner) when looking for boards. Ultra96V2 should appear now. Can close Vivado before finishing creation of new project. 


## Flash hard coded config binary to EVM board
1. Follow https://dev.ti.com/tirex/explore/node?node=ANHlPre7EOLunyygFnRRKg__VLyFKFf__LATEST which is also compatible with AOP version. 
Download the Industrial Toolbox and find the mentioned binary. Follow https://training.ti.com/hardware-setup-iwr6843aop for flashing instructions. 
Auto-detecting device did not work, enter manually (like in linked video).
2. EVM board should now be outputting data on the UART port. 

## Interpret EVM UART data stream
https://dev.ti.com/tirex/explore/content/mmwave_industrial_toolbox_4_8_0/labs/out_of_box_demo/common/docs/understanding_oob_uart_data.html
- An output packet is sent out every frame through the UART.
- Every packet consists of a Frame Header. Additional TLV items may also be sent each frame depending on the options enabled in the guiMonitor commmand and the number of detected objects.
- Each TLV item consists of a TLV header and value (payload) information.
- The length of the packet can depend on the number of detected objects and vary from frame to frame.
- The end of the packet is padded so that the total packet length is always multiple of 32 Bytes.

### Frame Header   

Length: 44 Bytes  
A Frame Header is sent at the start of each packet. Use the Magic Word to find the start of each packet.

| Value                                              | Type      | Bytes | Details
|----------------------------------------------------|-----------|-------|------------
| Magic Word                                         | uint16_t  | 8     | Output buffer magic word (sync word). It is initialized to {0x0102,0x0304,0x0506,0x0708}
| Version                                            | unint32_t | 4     | SDK Version represented as (MajorNum x 2^24 + MinorNum x 2^16 + BugfixNum x 2^8 + BuildNum)
| Total Packet Length                                | unint32_t | 4     | Total packet length including frame header length in Bytes
| Platform                                           | unint32_t | 4     | Device type (ex 0xA6843 for IWR6843 devices)
| Frame Number                                       | unint32_t | 4     | Frame number (resets to 0 when device is power cycled or reset. Not when sensor stop/start is issued.)
| Time [in CPU Cycles]                               | unint32_t | 4     | Time in CPU cycles when the message was created.
| Num Detected Obj                                   | unint32_t | 4     | Number of detected objects (points) for the frame
| Num TLVs                                           | unint32_t | 4     | Number of TLV items for the frame.
| Subframe Number                                    | unint32_t | 4     | 0 if advanced subframe mode not enabled, otherwise the sub-frame number in the range 0 to (number of subframes - 1)

----------------------------------------------------------------


### TLV Header   
Length: 8 Bytes  
* The number of TLVs in the frame packet is extracted from the Frame Header.  
* For each TLV in the packet, there is a TLV Header containing Type and Length information. 
  * The Type identifier indicates what kind of information is contained in the playload.  
  * The Length value gives the length of the payload. 

| Value                                              | Type      | Bytes | Details
|----------------------------------------------------|-----------|-------|-----------
| Type                                               | unint32_t | 4     | Indicates types of message contained in payload. 
| Length                                             | unint32_t | 4     | Length of the payload in Bytes (does not include length of the TLV header)  

----------------------------------------------------------------

### TLV Type Identifier  

The parameters in the CLI command **guiMonitor** are used to enable or disable whether the TLV type is included in the output frame packet.
The parameters are as follows: `guiMonitor <subFrameIdx> <detected objects> <log magnitude range> <noise profile> <rangeAzimuth(Elevation)HeatMap> <rangeDopplerHeatMap> <statsInfo>`  


For the Out of Box demo, if type contains the following value then the payload contains the information listed under value type and should be parsed accordingly.

TYpe Identifier | Value Type                        | Conditions for output
-----------|----------------------------------------------------------
1          | Detected Points                        | `<detected objects>` is set to 1 or 2 AND there are detected objects for the frame, else this type is not sent for that frame
2          | Range Profile                          | `<log magnitude range>` is set to 1; occurs every frame 
3          | Noise Floor Profile                    | `<noise profile>` is set to 1; occurs every frame 
4          | Azimuth Static Heatmap                 | `<rangeAzimuth(Elevation)HeatMap>` is set to 1 AND demo is not for AOP or ODS which use AOA2D; occurs every frame
5          | Range-Doppler Heatmap                  | `<rangeDopplerHeatMap>` is set to 1; occurs every frame 
6          | Statistics (Performance)               | `<statsInfo>` is set to 1; occurs every frame 
7          | Side Info for Detected Points          | `<detected objects>` is set to 2 AND there are detected objects for the frame, else this type is not sent for that frame 
8          | Azimuth/Elevation Static Heatmap       | `<rangeAzimuth(Elevation)HeatMap>` is set to 1 AND demo is for AOP or ODS which use AOA2D; occurs every frame 
9          | Temperature Statistics                 | `<statsInfo>` is set to 1; occurs every frame 

----------------------------------------------------------------

### TLV Payload
 
#### **Detected Points**       
Type: 1                                
Length: 16 Bytes x Num Detected Obj  
Value: Array of detected points. Each point is represented by 16 bytes giving position and radial Doppler velocity as shown in the table below:
 
| Value                                              | Type      | Bytes |
|----------------------------------------------------|-----------|-------|
| X [m]                                              | float     | 4     |
| Y [m]                                              | float     | 4     |
| Z [m]                                              | float     | 4     |
| doppler [m/s]                                      | float     | 4     |
 
#### **Side Info for Detected Points**   
Type: 7  
Length: 4 Bytes x Num Detected Obj   
Value: The payload consists of 4 bytes for EACH point in the point cloud. The values for snr and noise are determined as described in the doxygen documentation for the mmWave SDK Demo. 
 
| Value                                              | Type      | Bytes |
|----------------------------------------------------|-----------|-------|
| snr [dB]                                           | uint16_t  | 2     |
| noise [dB]  

(all types at link above)

## Ultra96 FPGA JTAG programming
1. Follow instructions at https://www.hackster.io/BryanF/ultra96-v2-vivado-2020-2-basic-hardware-platform-6b32b8 to create basic hardware platform.
2. Then follow https://www.hackster.io/BryanF/ultra96-v2-vitis-2020-2-hello-world-from-arm-a53-2d952a to use hardware platform to create simple program to configure PS (needed to get clock to PL).
3. From Vitis, run simple application as debug to initialize PS - and keep it running.
![Alt text](https://github.com/nhma20/ultra96_mmwave_interface/blob/main/Pictures/Screenshotfrom2021-07-1318-04-27.png?raw=true)
5. From Vivado, program device with .bit-file to initialize FPGA with custom design, while PS clock is routed to PL. (https://forums.xilinx.com/t5/Xilinx-Evaluation-Boards/how-to-use-PS-Clock-for-PL-logic/m-p/783506/highlight/true#M15628)
6. FPGA can be re-programmed as usual now, and the PL clock will function as long as Vitis debug keeps PS configured.


## Prebuilt hardware platform 
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


## MISC
1. The ISK antenna has 15 degree angular resolution compared to the AOP's 30 degree in azimuth. The ISK's antenna also has a larger gain allowing it to see further. Please see the antenna database for more specifications.
2. UART: The EVMs are configured for 3V3.
3. HCC: hard coding config of board.

## TODO
1. :green_circle: Configure ultra96 board, simple I/O program
2. :green_circle: Figure out how to use PS/PL clock without making new platform each time.
3. :green_circle: Find suitable interface on IWR6843AOPEVM board -> hard coded config keeps UART data streaming while board is powered
5. :green_circle: How to send config file from FPGA to IWR6843AOPEVM -> HCC (hard code config) which makes board automatically stream data once turned on
6. :green_circle: How to request/receive data from IWR6843AOPEVM -> Automatically streams data on UART port when HCC'd
7. :yellow_circle: Interpret UART packages automatically sent from EVM
8. :yellow_circle: Interface with EVM UART from FPGA
9. :yellow_circle: Store "points" in BRAM

