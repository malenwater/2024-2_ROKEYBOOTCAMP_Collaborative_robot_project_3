# 2024-2_ROKEYBOOTCAMP_Collaborative_robot_project_3
rivz, 가제보의 시뮬레이션으로 활용하여 가상의 시나리오(2 대의 터틀봇3를 이용한 화성 탐사 시나리오)를 만들고, 시나리오를 해결(화성 순찰, 임무 수행, 복귀)하는 프로젝트를 진행하였다.

## 🚀 Use ROS2 Package
총 3개의 터미널을 사용합니다. 해당 패키지는 turtlebot3 패키지를 필요로 합니다.
기본 사항

      git clone https://github.com/malenwater/2024-2_ROKEYBOOTCAMP_Collaborative_robot_project_3.git ~/2024-2_ROKEYBOOTCAMP_Collaborative_robot_project_3
      cd ~/2024-2_ROKEYBOOTCAMP_Collaborative_robot_project_3/ws
      . build.sh
      . source.sh

해당 과정은 깃을 본인 컴퓨터에 가져오고, colcon build와 source install/setup.bash를 하는 과정을 sh 파일로 만든 것입니다.
<br>
아래 3개의 과정을 순서대로 진행하면 본 프로젝트의 화성 탐사 시나리오를 진행할 수 있습니다. 해당 pwd(경로)는 ws 내부에서 실행되어야합니다.

<br>
1. 가제보 시뮬레이션을 세팅합니다.    

      . source.sh
      . com_sh/ui_start.sh 

<br>
2. 메인 서버를 실행합니다.

      . source.sh
      . com_sh/ui_start.sh
      
<br>
3. UI 를 실행합니다.

      . source.sh
      . com_sh/ui_start.sh 

<br>
위 3개의 작업을 하면 UI를 조작하여 화성 탐사 시나리오를 순찰, 정지, 복귀, 궤도 폭격, 광물 회수, 광물 생성 버튼을 통해 실험할 수 있습니다.<br>
예시 시나리오)<br>
    1. `순찰` 버튼을 누른다.<br>
    2. 로봇 2대가 순찰을 시작한다.(만약 순찰하지 않으면 한번더 버튼을 누른다.)<br>
    3. 로봇이 광물을 발견하면 광물로 이동한다.<br>
    4. 어느정도 가까워지면 로봇이 멈추고 동료 순찰 로봇을 부른다.<br>
    5. 동료 로봇이 발견한 로봇 근처에 온다.<br>
    6. 사용자(본인)이 로봇의 이미지를 보고, 광물을 포격할지 회수할지를 `궤도 포격`, `광물 회수` 버튼을 통해 선택하여 누른다.<br>
    7. 광물이 사리진다.<br>
    8. 로봇의 `복귀` 버튼을 눌러 두 로봇을 베이스로 복귀 시킨다.<br>

<br>
## ✨ Key Features

**시뮬레이션**

- 가제보와 rivz를 통해서 라이더, 센서, 로봇 링크와 관절, 맵 설계를 할 수 있다.

**가상 시나리오**

- 설계한 로봇과 맵, 센서를 이용해서 가상에서 시나리오를 만들어서 실행할 수 있다.

<br>

### Source
  - [터틀봇3 공식 사이트](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
  - 
##
|이름|문제점|해결 방안|역할|
|---|---|---|---|
|이선우|더미|더미|더미|
