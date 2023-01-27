
To communicate between ROS and teensy,(Serial should be commented out.)
reboot the teensy first and,

```
rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=4000000
```

```
rostopic echo /psyonic_hand_vals