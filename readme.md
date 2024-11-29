base on Benfang setup with a haptic device (Omni Touch), an UR5-cb3 robot arm, and a sunrise F/T sensor.

 - Omni Touch USB
 - robot arm 192.168.0.100
 - F/T sensor TODO
 - Host Machine 192.168.0.2
  
roslaunch:
```bash
alias start_robot1="roslaunch ur_robot_driver ur5_bringup.launch limited:=true robot_ip:=192.168.0.100"
```


