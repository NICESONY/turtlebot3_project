

### run computer

```

ros2 launch turtlebot3_gazebo smart_farm_model.launch.py use_sim_time:=true

ros2 launch turtlebot3_navigation2 smart_farm_navigation2_simul.launch.py use_sim_time:=true

ros2 run my_turtlebot_pkg test_web

http://192.168.0.102:8080/

ros2 run turtle3_project smart_farm_manager_simul


## 베터리 상태에 따른 시뮬레이션 실험을 위한 가짜 베터리 토픽 Pub node

ros2 run turtle3_project fake_battery 


```
