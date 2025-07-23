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

### 👥 팀구성 : 손건희, 박현준, 박진우, 곽정미


## 📅 프로젝트 일정


|날짜|조원|시간|작업 내용|
|:---:|:---:|:---:|:---|
|2025/07/21 |모두 참여|하루 종일(오전 & 오후)|1. 주제 정하기 <br> 2. github 세팅<br> 3. 계획서 작성<br> 4. PPT 작성 중<br> 5. gazebo map 초기 세팅|
| <br>| | | |
|2025/07/22 |박현준|오전|웹과 ROS 연동 중<br> |
|2025/07/22 |곽정미|오전|추가적인 모터 설치 중|
|2025/07/22 |박진우|오전|아르코 마커 진행 중|
|2025/07/22 |손건희|오전|베터리 전압 토픽 확인 중 + 연결 + turtlebot3 mash & urdf change 보류|
|2025/07/22 |박현준|오후|웹과 토픽 확인|
|2025/07/22 |곽정미|오후|추가적으로 모터 설치 후 ROS와 연결|
|2025/07/22 |박진우|오후|아르코 마커로 움직임 완료|
|2025/07/22 |손건희|오후|사람 따라가기 완료, 일정 거리 유지 가능|
| |<br> | | |
|2025/07/23 |금일 목표|오전 & 오후|1. 웹에서 토픽을 보내서 움직이게하기 <br> 2. 특정 지점 도착하몀 모터 움직이게 하기 <br> 3. 베터리 일정 이라면 이동하게 하기|
|2025/07/23 |박현준|오전|웹 인터페이스 디자인 및 DB 설정 |
|2025/07/23 |곽정미|오전|서브모터를 ROS와 연결하기 위한 서비스 만드는 중|
|2025/07/23 |박진우|오전|아르코 마커로 특정 마커일 경우만 서브모터를 움직이도록 만드는 중|
|2025/07/23 |손건희|오전|베터리 상태에 따른 충전소로 위치 코드 작성 중 + 실제에서 test 필요|
|2025/07/23 |박현준|오후|웹과 디자인과 ROS 코드와 연동 협력 작업 진행|
|2025/07/23 |곽정미|오후|아르코 마커와 서브모터와 함께 협업 진행 + 특정 아르코 마커에만 바구니를 내려서 짐 옮기기 기능|
|2025/07/23 |박진우|오후|아르코 마커와 서브모터와 함께 협업 진행|
|2025/07/23 |손건희|오후|웹과 디자인과 ROS 코드와 연동 협력 작업 진행 + 최종적으로 클릭한 지점에서 멈춰서 수확 후 특정 지점으로 이동 가능|
| | <br>| | |
|2025/07/24 |박현준|오전| |
|2025/07/24 |곽정미|오전| |
|2025/07/24 |박진우|오전||
|2025/07/24 |손건희|오전||
|2025/07/24 |박현준|오후|중앙정렬|
|2025/07/24 |곽정미|오후|중앙정렬|
|2025/07/24 |박진우|오후|중앙정렬|
|2025/07/24 |손건희|오후|중앙정렬|
| | <br>| | |
|2025/07/25 |모두 참여|중앙정렬|발표, PPT 완성 예정|

## ❓ 질문사항

- 모터를 달아서 특정 위치에서 모터를 작동 시켜서 기울이고 싶은데 어떻게 할지??

### 🗓️ 서류 링크

- [프로젝트 링크](https://docs.google.com/presentation/d/1j1xKGVV8NkETSxIdAJgCO2qdbaPO2zO-178N694P2_A/edit?slide=id.g36f529a8a23_0_5#slide=id.g36f529a8a23_0_5)
- [계획서](https://docs.google.com/document/d/1M6PhMZ9piBhJVs8vatvDK25QiM19CuVPqjAv2bpU3-8/edit?tab=t.0)
- [PPT](https://docs.google.com/presentation/d/1NAYQ3nXiv2y0jL_QtZ-ypov38d0UsTY2KapHy4Q52Sg/edit?slide=id.g370cc601211_0_15#slide=id.g370cc601211_0_15)

## 📅 구현할 기능 & 완료한 기능

|날짜|조원|구현 할 기능|
|:---:|:---:|:---|
|2025/07/21 |손건희|계획서 작성 OK|
|2025/07/21 |손건희|github 초기 세팅 완료 OK|
|2025/07/22 |손건희|시뮬레이션에서도 뒤쪽 바구니 설치를 구현하기 위한 STL파일 수정 및 URDF 파일 수정 보류 X|
|2025/07/22 |손건희|SLAM 기반 위치 이동 OK|
|2025/07/22 |박진우|인벤터를 이용해서 환경 만들고 + 가제보와 연결 완료 OK|
|2025/07/22 |손건희|베터리 토픽 받고 일정 이하의 경우 이동하게 하기|
|2025/07/21 |???|SLAM 기반 위치 이동 + 웹 연계|
|2025/07/21 |???|특정 전압 이하의 경우 충전 스테이션으로 이동 기능 구현|
|2025/07/21 |???|AR 테그를 바탕으로 지정된 곳에 과일 내리기 기능 + 옮겨서 장난 치기 가능할 듯|
|2025/07/21 |???|사람 따라 다니게 하면서 수확하기 사람이 있으면 멈추기|
|2025/07/21 |???|자동 수확 기능?? 구간 순회 하면서 최종 목표에 도착 기능 (SLAM)|
|2025/07/21 |???|전체 디버깅|
|2025/07/21 |???|시뮬레이션 영상 찍기|
|2025/07/21 |???|실제 구현 영상 찍기|
|2025/07/21 |???|PPT 작성|



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

## git commad


### 처음에는 항상 상대방의 수정사항을 먼저 받아줘야합니다.

```
git pull
```

### 내가 수정한 것을 깃 원격 즉 웹상에 올리고 싶을때

```
git add .
git commit -m "update"
git push
```


## Project Notion link
- 

## Project github command list
- [link](https://github.com/NICESONY/turtlebot3_project/tree/main/git_commad)


## pull request & code review & make branch and merge


## 진행과정 중 필요한 준비물

- 먼저 터틀봇 연결 확인을 위해서 모니터와 모니터를 연결할 수 있는 HTMI 포트가 필요함 + 이것을 이용해서 같은 네트워크에 연결해서 로봇에 접속하고자 함
- hi
- hi 2

## [map_image](https://github.com/NICESONY/turtlebot3_project/blob/main/Documentation/map_image/Readme.md)

