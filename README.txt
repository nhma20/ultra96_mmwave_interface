Example of hard-coded sensor config for 14xx using mmWave SDK 1.2 mmw demo

Steps to update mmWave SDK 1.2 14xx demo to use example hard-coded config:
1. Rename/backup existing cli.c source file in mmwave_sdk_01_02_00_05\packages\ti\utils\cli\src directory
2. Place new cli.c file in directory
3. Re-build cli library using gmake as described in mmWave SDK user guide
  a. Make sure to update setenv.bat with correct device (i.e. iwr14xx) before executing it (C:\ti\mmwave_sdk_02_01_00_04\packages\scripts\windows)
  b. Make sure to run 'gmake clean' and then 'gmake all' in the mmwave_sdk_01_02_00_05\packages\ti\utils\cli directory
4. Re-build 14xx mmw project by right-clicking on project in CCS and
performing "Re-build" as described in the user guide on the TI Resource
Explorer

NOTE: If replacing/updating the example sensor config given in the cli.c file,
make sure to use a config that was generated for the desired device and mmWave
SDK version.
