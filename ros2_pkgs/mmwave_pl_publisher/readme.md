Publishes data from the mmWave radar device as it is transfered from PL to PS via BRAM and to PC via UART.

Running:
ros2 run mmwave_pl_publisher mmwave_publisher_node --ros-args -p port:='/dev/ttyUSB1' -p baud:=115200 -p n_points:=8
