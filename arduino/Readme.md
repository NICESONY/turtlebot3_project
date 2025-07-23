### robot

```


mkdir root_ws
cd root_ws
mkdir src
cd src
ros2 pkg create tab_pkg --build_type ament_python

# 터틀봇 전반적인 기능 실행
ros2 launch turtlebot3_bringup robot.launch.py

# 카메라 실행
ros2 run v4l2_camera v4l2_camera_node \
  --ros-args -r __ns:=/camera \
            -p image_size:="[640,480]"
            

 ros2 run tab_pkg manual_dump_service_server 

```


## local


```
mkdir local_ws
cd local_ws
mkdir src
cd src
ros2 pkg create aruco_detector_pkg --build_type ament_python

ros2 run aruco_detector_pkg test_aruco

```
