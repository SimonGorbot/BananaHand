### How to Run Serial Node
- start mcu code: `cargo run -p uart_test`
- start ros code:
```
cd software/ros
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

ros2 launch banana_bringup bringup.launch.py port:=/dev/ttyACM0 baud:=115200
```
- test ros code:
```
ros2 topic pub --once /tx_positions std_msgs/msg/Float32MultiArray \
"{data: [1,2,3,4,5,6,7,8]}"
```
- echo topic for verification: `ros2 topic echo /rx_positions`