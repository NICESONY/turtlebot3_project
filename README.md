# 🚜 Smart Farm Transport Robot



### 🗓️ 프로젝트 기간 : 2025/07/21 ~ 2025/07/25



## 🎯 주제 : 스마트팜 자동 운송 로봇 및 충전 스테이션 연계 시스템 



### 🤖 주요 기능 : Radir SLAM, NAV2, Vision, Gazebo, 

- 1. 특정 베터리 이하일 경우 충전 구역으로 이동 --> slam +  nav2  == Sim O  Real X
- 2. 웹에서 클릭시 특정 구역으로 이동 + slam +  nav2 == Sim O  Real X
- 3. 웹에서 클릭 시 자율 수확 모드(사람을 따라다니면서 수확 진행) + slam  + vision + nav2 == Sim O  Real X
- 4. 특정 장소로 이동해서 수확물 특정 박스에 옮기기 + slam + vision + nav2 + 모터 연결하여 서비스 만들어서 기능 구현 + 특정 아루코 마커 아닐 시 장소 찾아서 놓기  == Sim O  Real  X
  #### 번외
- 2개 이상의 터틀봇 이용해서 다른 로봇이 옮기면 알아서 자동으로 배치
- 특정 구역에 과일 옮기기 (AR 테그 사용?? 특정 ar 테그의 경우 그곳에 상자 기울이기)

  
### 준비물 : 아두이노 보드, SG90 서보모터, 골판지 or 화이트 보드

### 시연영상 찍어서 보내기 + 터틀봇 정리하고 앞에 두기

### 👥 팀구성 : 손건희, 박현준, 박진우, 곽정미


## 📅 [프로젝트 일정](https://github.com/NICESONY/turtlebot3_project/tree/main/Documentation/plan)


## ❓ 질문사항

- 모터를 달아서 특정 위치에서 모터를 작동 시켜서 기울이고 싶은데 어떻게 할지??

### 🗓️ 서류 링크

- [프로젝트 링크](https://docs.google.com/presentation/d/1j1xKGVV8NkETSxIdAJgCO2qdbaPO2zO-178N694P2_A/edit?slide=id.g36f529a8a23_0_5#slide=id.g36f529a8a23_0_5)
- [계획서](https://docs.google.com/document/d/1M6PhMZ9piBhJVs8vatvDK25QiM19CuVPqjAv2bpU3-8/edit?tab=t.0)
- [PPT](https://docs.google.com/presentation/d/1NAYQ3nXiv2y0jL_QtZ-ypov38d0UsTY2KapHy4Q52Sg/edit?slide=id.g370cc601211_0_15#slide=id.g370cc601211_0_15)

## 📅 구현할 기능 & 완료한 기능


 ### git clone 하는 방법
```

## make git clone

- open terminer
- cd ~
- mkdir project
- cd project
- git clone https://NICESONY:{tokenkey}@github.com/NICESONY/turtlebot3_project.git
- code .
```

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

## [Project github command list](https://github.com/NICESONY/turtlebot3_project/tree/main/git_commad)

## pull request & code review & make branch and merge

# [map_image](https://github.com/NICESONY/turtlebot3_project/blob/main/Documentation/map_image/Readme.md)


-------------

## 🎯  폴더 설명 


- 특정 폴더에 뭐가 들어있는지 설명하는 부분
