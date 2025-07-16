## 로봇 ssh 주소 
- ros25@ros25-virtual-machine:~$ ssh ubuntu@192.168.0.83
- password : aa
- 로봇 : ubuntu@ubuntu16:~$ ros2 launch turtlebot3_bringup robot.launch.py
- local : ros2 run turtlebot3_teleop teleop_keyboard
- 
### 막히는 경우
- local soucre ~/.bashrc
- ros2 daemon stop && ros2 daemon start
