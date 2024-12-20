base on Benfang setup with a haptic device (Omni Touch), an UR5-cb3 robot arm, and a sunrise F/T sensor.

 - Omni Touch USB
 - robot arm 192.168.0.100
 - F/T sensor TODO
 - Host Machine 192.168.0.2
  
roslaunch:
```bash
alias start_robot1="roslaunch ur_robot_driver ur5_bringup.launch limited:=true robot_ip:=192.168.0.100"
```

### problem
```bash
 ImportError: /lib/x86_64-linux-gnu/libp11-kit.so.0: undefined symbol: ffi_type_pointer, version LIBFFI_BASE_7.0 #509 
```
### solution
https://github.com/ros-perception/vision_opencv/issues/509, downgrade the python in conda to 3.8.10

```bash
 conda install python=3.8.10
```

