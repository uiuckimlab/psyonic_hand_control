Install dependencies
```
sudo apt-get install ros-noetic-rosserial-arduino
```

To communicate between ROS and teensy,(Serial should be commented out.)
reboot the teensy first and,

```
rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=4000000
```

```
rostopic echo /psyonic_hand_vals


rostopic pub /psyonic_controller std_msgs/Float32MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data:
- 0
- 0
- 0
- 0
- 0
- 0
"