base on Benfang setup with a haptic device (Omni Touch), an UR5-cb3 robot arm, and a sunrise F/T sensor.

 - Omni Touch USB
 - robot arm 192.168.0.100
 - F/T sensor TODO
 - Host Machine 192.168.0.2
  
roslaunch:
```bash
alias start_robot1="roslaunch ur_robot_driver ur5_bringup.launch limited:=true robot_ip:=192.168.0.100"
```


## Setup
Once the package is cloned or updated, run 
```bash
pip install -e .
```

## Q&A

### Q1   
```bash
 ImportError: /lib/x86_64-linux-gnu/libp11-kit.so.0: undefined symbol: ffi_type_pointer, version LIBFFI_BASE_7.0 #509 
```
#### solution 
https://github.com/ros-perception/vision_opencv/issues/509, downgrade the python in conda to 3.8.10

```bash
 conda install python=3.8.10
```

### Q2
```bash
WARNING: disk usage in log directory [/home/rmqlife/.ros/log] is over 1GB.
It's recommended that you use the 'rosclean' command.
```
#### solution
Run the following command to clean the log directory:
```bash
rosclean purge
```
This will remove old logs that are no longer needed and help free up space.


### Q3

attribute error or topic missing
```bash
AttributeError: 'MyFTSensor' object has no attribute 'force'
```
#### solution
make sure run the following and the rosnode name is available for the system
```python 
rospy.init_node('test_node', anonymous=True)
```


### Q4
update your Git repository URL to use SSH
```bash
git remote set-url origin git@github.com:rmqlife/ur5_ros_util.git
```


## Quick Notes

#### DiffTactile: A Physics-based Differentiable Tactile Simulator for Contact-rich Robotic Manipulation
https://difftactile.github.io/

