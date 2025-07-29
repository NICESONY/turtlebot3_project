Sure! Here's the English version of your project summary:

---

# 🚜 Smart Farm Transport Robot

### 🗓️ Project Duration: 2025/07/21 \~ 2025/07/25

## 🎯 Topic: Smart Farm Autonomous Transport Robot and Charging Station Integration System

### 🤖 Key Features: Radir SLAM, NAV2, Vision, Gazebo

1. Automatically moves to the charging zone when the battery falls below a certain level
2. Moves to a designated zone upon a click from the web interface
3. Autonomous harvesting mode activated via web click (follows a person and assists with harvesting)
4. Moves to a specific location and transfers harvested crops to a designated box

   #### Extra Feature:

   * When using two or more TurtleBots, the system automatically arranges tasks based on which robot is available for transport

### 👥 Team Members: Kunhee Son, Hyunjun Park, Jinwoo Park, Jungmi Kwak



## 📅 [Project Schedule](https://github.com/NICESONY/turtlebot3_project/tree/main/Documentation/plan)


## ❓ Q & A


### 🗓️ Document Links

- [계획서](https://docs.google.com/document/d/1M6PhMZ9piBhJVs8vatvDK25QiM19CuVPqjAv2bpU3-8/edit?tab=t.0)
- [PPT](https://docs.google.com/presentation/d/1NAYQ3nXiv2y0jL_QtZ-ypov38d0UsTY2KapHy4Q52Sg/edit?slide=id.g370cc601211_0_15#slide=id.g370cc601211_0_15)


## ⚙️ git clone 
```

open terminer
cd ~
mkdir project
cd project
git clone https://NICESONY:{tokenkey}@github.com/NICESONY/turtlebot3_project.git
cd turtlebot3_project
git add .
git commit -m "initial_commit"
git push

code .
```

## ⚙️ install 
```
sudo apt update
sudo apt install \
  ros-humble-nav2-bringup \
  ros-humble-navigation2 \
  ros-humble-nav2-amcl \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-nav2-planner \
  ros-humble-nav2-controller \
  ros-humble-nav2-behavior-tree


sudo apt update
sudo apt install ros-humble-cartographer ros-humble-cartographer-ros
```

## 🎯  여기부터 디버깅 완벽히 잡고 스프링 이관 설치

- 새로운 환경에서 스프링 서버 및 ros 연계
- git clone 후 spwan 여러개 생기는 문제 해결 필요
- 서버 DB 테이블 다른 것 만들어 볼 것



## 📊 [Project github command list](https://github.com/NICESONY/turtlebot3_project/tree/main/git_commad)


## 🗺️ [map_image](https://github.com/NICESONY/turtlebot3_project/blob/main/Documentation/map_image/Readme.md)

-------------
## 📊  [기술서](https://github.com/NICESONY/turtlebot3_project/blob/main/Documentation/Project_Technical_Specification/%ED%94%84%EB%A1%9C%EC%A0%9D%ED%8A%B8_%EA%B8%B0%EC%88%A0%EC%84%9C_%EC%96%91%EC%8B%9D(%EC%8A%A4%EB%A7%88%ED%8A%B8%ED%8C%9C%20%EC%9E%90%EB%8F%99%20%EC%9A%B4%EC%86%A1%20%EB%A1%9C%EB%B4%87%20%EB%B0%8F%20%EC%B6%A9%EC%A0%84%20%EC%8A%A4%ED%85%8C%EC%9D%B4%EC%85%98%20%EC%97%B0%EA%B3%84%20%EC%8B%9C%EC%8A%A4%ED%85%9C).pdf)
-------------


## 🤖 [Youtube Video](https://youtu.be/RD2XwL_64qI)
