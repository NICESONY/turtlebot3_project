# 🚜 Smart Farm Transport Robot



### 🗓️ 프로젝트 기간 : 2025/07/21 ~ 2025/07/25



## 🎯 주제 : 스마트팜 자동 운송 로봇 및 충전 스테이션 연계 시스템 



### 🤖 주요 기능 : Radir SLAM, NAV2, Vision, Gazebo, 

- 1. 특정 베터리 이하일 경우 충전 구역으로 이동 
- 2. 웹에서 클릭시 특정 구역으로 이동 
- 3. 웹에서 클릭 시 자율 수확 모드(사람을 따라다니면서 수확 진행) 
- 4. 특정 장소로 이동해서 수확물 특정 박스에 옮기기
  #### 번외
- 2개 이상의 터틀봇 이용해서 다른 로봇이 옮기면 알아서 자동으로 배치
- 특정 구역에 과일 옮기기 (AR 테그 사용?? 특정 ar 테그의 경우 그곳에 상자 기울이기)

  
### 👥 팀구성 : 손건희, 박현준, 박진우, 곽정미


## 📅 [프로젝트 일정](https://github.com/NICESONY/turtlebot3_project/tree/main/Documentation/plan)


## ❓ 질문사항


### 🗓️ 서류 링크

- [프로젝트 링크](https://docs.google.com/presentation/d/1j1xKGVV8NkETSxIdAJgCO2qdbaPO2zO-178N694P2_A/edit?slide=id.g36f529a8a23_0_5#slide=id.g36f529a8a23_0_5)
- [계획서](https://docs.google.com/document/d/1M6PhMZ9piBhJVs8vatvDK25QiM19CuVPqjAv2bpU3-8/edit?tab=t.0)
- [PPT](https://docs.google.com/presentation/d/1NAYQ3nXiv2y0jL_QtZ-ypov38d0UsTY2KapHy4Q52Sg/edit?slide=id.g370cc601211_0_15#slide=id.g370cc601211_0_15)

## 🖥️ [구현할 기능 & 완료한 기능](https://github.com/NICESONY/turtlebot3_project/tree/main/Documentation/Note)


## ⚙️ git clone 하는 방법
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



## 📊 [Project github command list](https://github.com/NICESONY/turtlebot3_project/tree/main/git_commad)

## pull request & code review & make branch and merge

## 🗺️ [map_image](https://github.com/NICESONY/turtlebot3_project/blob/main/Documentation/map_image/Readme.md)

-------------
## 📊  [기술서](https://github.com/NICESONY/turtlebot3_project/blob/main/Documentation/Project_Technical_Specification/%ED%94%84%EB%A1%9C%EC%A0%9D%ED%8A%B8_%EA%B8%B0%EC%88%A0%EC%84%9C_%EC%96%91%EC%8B%9D(%EC%8A%A4%EB%A7%88%ED%8A%B8%ED%8C%9C%20%EC%9E%90%EB%8F%99%20%EC%9A%B4%EC%86%A1%20%EB%A1%9C%EB%B4%87%20%EB%B0%8F%20%EC%B6%A9%EC%A0%84%20%EC%8A%A4%ED%85%8C%EC%9D%B4%EC%85%98%20%EC%97%B0%EA%B3%84%20%EC%8B%9C%EC%8A%A4%ED%85%9C).pdf)
-------------

## 🎯  폴더 설명 


- 특정 폴더에 뭐가 들어있는지 설명하는 부분


## 🤖 [시연 영상](https://youtu.be/RD2XwL_64qI)
