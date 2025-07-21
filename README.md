## 🚜 Smart Farm Transport Robot



## 🗓️ 프로젝트 기간 : 2025/07/21 ~ 2025/07/25

## 🎯 주제 : 스마트팜 자동 운송 로봇 및 충전 스테이션 연계 시스템 



## 🤖 주요 기능 : Radir SLAM, NAV2, Vision, Gazebo, 

- 특정 베터리 이하일 경우 충전 구역으로 이동 ar 테그로 특정 구역에서 정지
- 웹에서 클릭시 충전 구역으로 이동
- 특정 구역에 과일 옮기기 (AR 테그 사용?? 특정 ar 테그의 경우 그곳에 상자 기울이기)
- 웹에서 클릭시 특정 지점에서 과일 재배 도움 
- 웹에서 클릭시 특정 지점에서 자동화 모드로 과일 도움 - 카메라로 특정 인물이 보일 시 옆에서 대기하거나?? 등등

## 준비물 : 아두이노 보드, SG90 서보모터, 골판지 or 화이트 보드

## 👥 팀구성 : 손건희, 박현준, 박진우, 곽정미


## 📅 프로젝트 일정


|날짜|조원|시간|작업 내용|
|:---:|:---:|:---:|:---:|
|2025/07/21 |모두 참여|주제 정하기, github 세팅, 계획서, PPT 작성 중|
|왼쪽정렬|오른쪽정렬|중앙정렬|
|왼쪽정렬|오른쪽정렬|중앙정렬|

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
|2025/07/21 |???|SLAM 기반 위치 이동|
|2025/07/21 |???|SLAM 기반 위치 이동 + 웹 연계|
|2025/07/21 |???|특정 전압 이하의 경우 충전 스테이션으로 이동 기능 구현|
|2025/07/21 |???|AR 테그를 바탕으로 지정된 곳에 과일 내리기 기능 + 옮겨서 장난 치기 가능할 듯|
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
-


## pull request & code review & make branch and merge


## 진행과정 중 필요한 준비물

- 먼저 터틀봇 연결 확인을 위해서 모니터와 모니터를 연결할 수 있는 HTMI 포트가 필요함 + 이것을 이용해서 같은 네트워크에 연결해서 로봇에 접속하고자 함
- hi
- hi 2
