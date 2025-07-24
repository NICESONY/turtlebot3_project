

## run robot(ssh)

```
ros2 launch turtlebot3_bringup robot.launch.py
ros2 launch turtlebot3_bringup turtlebot3_state_publisher.launch.py 
```



##  run computer (slam)

```
ros2 launch turtlebot3_navigation2 smart_farm_navigation2.launch.py 
ros2 run turtle3_project smart_farm_manager_real
```

##  run computer (vision)

```
ros2 launch yolo_follower follow_yolo.launch.py
ros2 run rqt_image_view rqt_image_view
```


## slam

```
ros2 launch turtlebot3_bringup robot.launch.py
ros2 launch turtlebot3_bringup turtlebot3_state_publisher.launch.py

ros2 launch turtlebot3_cartographer cartographer.launch.py
ros2 run turtlebot3_teleop teleop_keyboard 
```

## nav

```
ros2 launch turtlebot3_bringup robot.launch.py
ros2 launch turtlebot3_bringup turtlebot3_state_publisher.launch.py

ros2 launch turtlebot3_navigation2 navigation2.launch.py 
ros2 run turtlebot3_teleop teleop_keyboard 

ros2 run nav2_map_server map_saver_cli -f ~/map
```
