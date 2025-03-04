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

## 진행한 프로젝트 리뷰
- 해당 과제에서는 로봇 2대를 이용하여 소환하고, 각각 nav2 기능과 서로 호출할 수 있는 등, 여러 로봇 간의 상호작용을 할 수 있는 시나리오가 필요했다. 그러므로 본 프로젝트에서는 화성 탐사 로봇 시나리오로 각각의 순찰 로봇이 순찰하고, 광물을 발견할 시 서로 호출한다. 또한 사용자가 해당 로봇의 화면을 보고 광물의 파괴와 회수 유무를 선택할 수 있다. 마지막으로 임무를 완수한 로봇은 복귀 및 정지가 가능한 것으로 시나리오를 구성하였다.
<p align="center">
  <img src="https://github.com/user-attachments/assets/1cf74a27-b9d3-4419-a8c6-15a9338d760d" alt="시나리오에 대한 전체 구성도">
</p>
<p align="center"><b><시나리오에 대한 전체 구성도></b></p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/a190da32-670f-4064-85dd-e0bf28727c38" alt="시나리오에 필요한 시스템 설계">
</p>
<p align="center"><b><시나리오에 필요한 시스템 설계></b></p>
- 위 이미지는 각각 시나리오 전체 구성도와 설계도이다. 이를 기반으로 시나리오 환경 구축 및 시나리오에 맞는 코드를 기능별로 나눠 팀원들과 작업하여 통합해 프로젝트를 완성할 수 있었다.


## 겪었던 문제사항과 해결 방법들
- waffle_pi가 가제보에서 소환되지 않는 문제
  - 문제 : robot_state_publisher을 통해 topic으로 가제보에 waffle_pi를 소환할 경우, 소환되지 않았다.
  - 해결 : 해당 문제는 rviz2에 쏘는 robot_state_publisher과 가제보에 소환될 시 필요한 sdf 파일로 파일의 종류가 2개로 나뉘어져있다. 이를 확인하고, 각각에 알맞은 waffle_pi의 urdf와 sdf를 설정하여 waffle_pi를 소환할 수 있었다.
 - waffle_pi가 rviz2 상에 nav2로 동작하지 않는 문제
   - 문제 : waffle_pi의 소환은 되었지만 rviz2에 nav2가 동작하도록 구현하는 것에 문제가 있다.
   - 해결 방향 : 해당 문제는 waffle의 nav2에 대한 설정파일과 waffle_pi의 설정파일이 다른 문제가 있다. 그래서 waffle_pi로 설정파일을 바꾸었지만 동작하지 않았다. 이에 대해 파일을 까서 확인한 결과, 세부적으로 수정해야하는 부분들이 다수 있었다. 이를 일일이 확인하며 바꾸기엔 시간이 없기에 기존 waffle로 진행하기로 결정하였다.
- waffle이 가제보상에서 색깔이 변경되지 않는 문제
  - 문제 : waffle을 가제보상 소환할 때 urdf에서 색깔을 바꾸어도 변경되지 않는 문제가 있었다.
  - 해결 : 해당 문제는 가제보와 rviz2의 색깔 변경은 각각 다른 파일에서 이루어지기 때문에 gazebo에 띄우는 sdf 파일에 색깔을 적용한 결과 가제보상에서 로봇 색깔이 바뀌는 것을 확인할 수 있었다.
- 광석의 색깔이 변경되지 않는 문제
  - 문제 : xacro를 통해 광석을 만들고 소환하면 광석의 색깔이 변하지 않고 흰색으로 나오는 문제가 있었다.
  - 해결 : 기존 xacro를 통해서 소환하는 방식은 로봇을 통해 소환하는 방식이다. 즉 이는 객체가 아닌 로봇을 소환하여 계속해서 위치를 트랙킹하기 위한 소환 방식이므로, 이런 방식이 아닌 sdf파일 형식으로 소환할 경우, 가제보 상에 색깔이 바뀌는 것을 확인할 수 있었다.
- Flask를 이용한 화면 송출 문제
  - 문제 : 가제보 상 로봇이 송출하는 이미지를 플라스크로 하여 웹상에 띄우는 것은 가능하나 지속적으로 끊기고, 네트워크가 불안정할 경우 웹이 꺼지는 문제가 발생하였다.
  - 해결 방향 : 해당 문제는 네트워크 불안정 문제와 통신 문제로 인해 일어난 문제로 통신 문제의 경우 해결하기 어렵다고 판단하였다. 그러므로 Flask가 아닌 PyQt를 통해 이미지를 송출하고 버튼을 구현하도록 변경하였다.
- 로봇 팔을 추가한 로봇 모델 생성 문제
  - 문제 : 로봇 모델을 뽑은 후, 가제보와 rviz2로 연동시켜서 소환시 소환이 안되고, 센서와 카메라가 연동되지 않는다.
  - 해결 방향 : 해당 문제는 waffle에 매니퓰레이터로 로봇 팔에 대한 정보를 넣었을 때 동작하지 않은 문제이다. 해당 문제는 위에서 해결한 방식으로 해결한다면 되겠지만 시간상 소환과 매니퓰레이터를 동작시키는데 시간이 걸릴 것으로 판단하여 waffle로 시나리오를 진행하기로 하였다.
- nav2의 이동 명령 취소 문제
  - 문제 : waffle이 순찰 중에 광물을 발견할 경우, 순찰 모드에서 추적 모드로 풀려야하기 때문에 nav2 기능을 취소해야하는데 취소가 안된다.
  - 해결 : 해당 문제는 바로 앞에 nav2 점을 찍어서 해결할려고 했으나 잘 되지 않았다. 그래서 진행 중이던 action을 취소할 수 있기 때문에, 취소하는 명령을 내려서 nav2가 실행이 취소되도록 구현하였다. 
- 추적시 광물이 중앙으로 오지 않던 문제
  - 문제 : 순찰 로봇이 추적 모드일 경우에, 광물이 이미지의 정 중앙에 와야하는데, 광물 위치가 오른쪽으로 치우치는 문제가 있다.
  - 해결 : 해당 문제 이전에는 코드상 회전 값이 너무 커서 티텍팅하고 나서 회전하면, 광물이 화면 밖으로 나가버리는 문제가 있었는데 이는 회전 값을 적게 주어 해결하였다. 또한 치우치는 문제는 회전 방향이 반대로 코드가 작성되어 있어 이를 반대로 넣으니 해결 되었다.

## 🎥 Demo Video
- [터틀봇3 와플 2대의 화성 탐사 로봇 시나리오](https://youtu.be/gcoVvCEzZLw)
- [협동 로봇3 발표 영상](https://youtu.be/3Ix2u36OIlY)
      
### Source
  - [터틀봇3 공식 사이트](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
  - 해당 로키부트캠프의 강사님 코드

## 해당 프로젝트의 문제점과 해결방안
|이름|문제점|해결 방안|역할|
|---|---|---|---|
|이선우|1. 강사님이 주셨던 코드는 터틀봇3 와플을 소환하는 것이었기 때문에, 이를 터틀봇3 와플 파이로 변경해볼려고 했는데, 소환되지 않는 문제가 있었다.<br> 2. 강사님 코드의 와플 rviz2를 이용한 nav2 기능을 와플 pi에 적용할려고 했는데 되지 않았다. <br>3. 가제보에 urdf를 통해 소환하여 터틀 봇3 와플의 색깔을 변경하려고 했지만 되지 않았다.<br> 4. xacro를 통해 가제보에 소환할 경우, 광물의 색깔이 입혀지지 않는 문제가 있었다. <br> 5. 메인 서버에 명령을 받는 서브스크라이버가 가끔 동작하지 않아서 사용자 명령을 수행하지 않는다.|1. 강사님 코드를 분석한 결과, rviz2에 쏴주는 robot state publisher의 경우, urdf를 통해 쏴주지만, 가제보 소환의 경우 sdf로 변환하여 쏴주어야 한다. 그러므로 와플 pi의 urdf와 sdf를 각각 publisher, gazebo에 넣어준 결과 와플 파이를 소환할 수 있었다.<br> 2. 해당 와플 파이에 대한 nav2에 대한 yaml 파일이 존재한다. 해당 yaml은 nav2 설정파일로 AMCL, BT 네비게이터, 컨트롤러 서버, 로컬 및 글로벌 코스트맵, 맵 서버 & 맵 세이버, 플레너 서버, 회복 동작, 로봇 상태 퍼블리셔 웨이포인트 팔로워등이 설정되어 있다. 하지만 이 nav2 파일이 우리 프로젝트와 맞지 않아 수정해야하므로 시간이 많이 소요될 것으로 예상함에 따라 와플 pi가 아닌 기존 와플 모델을 사용해 시나리오를 진행하기로 하였다. <br>3. 해당 urdf의 색깔을 변경해도 가제보의 색깔이 변경되지 않는 문제는 앞선 문제 1에서 처럼 sdf 파일의 색깔을 변경하면 색깔이 변경되었다. 하지만 rviz2에 있는 로봇의 색깔은 urdf를 변경해도 변하지 않는데, 이러한 이유는 urdf내부에 dae라는 파일을 불러와서 mesh를 설정하기 때문에 색깔 옵션이 먹히지 않는 것이었다. 하지만 이 dae 파일을 변경하려고 했지만 너무 복잡하게 엃혀있어서 rviz2의 색깔을 변경하는 건 실패하였다.<br> 4. 장애물의 색깔을 또한 1번 문제와 마찬가지로 소환시 sdf가 아닌 urdf로 소환하기 때문에 색깔 정보가 손실되어 가제보에 색깔이 입혀지지 않았다. sdf파일로 색깔을 입히고 소환하면 색깔이 입혀져서 소환되었다. <br> 5. 해당 사용자 명령을 무시하는 문제는 메인 서버가 단일로 동작할 경우, 문제가 없지만 가제보에 명령을 내리는 것까지 포함할 경우, 서브스크라이버가 명령을 받지 못하는 경우가 생겼다. 이를 보아 쓰레드를 사용할 때 문제가 생긴 것으로 보이며, 차후에 쓰레드들을 가각각 잘 분리하여 명령이 수행이 안되는 경우가 없도록 해야한다. 시간 관계상 해결 없이 쭉 수행하였다. 또한 메인 서버와 가제보를 같이 실행할 경우, 이러한 경우가 심해지는 것으로 보아 컴퓨터 성능에 따른 쓰레드의 일하는 속도가 달라져 심해지는 것으로 판단된다. <br>|코드 통합, 시나리오 작성|
|이강태|1.  추적시 광물이 중앙으로 오지 않던 문제
  <br> - 문제 : 순찰 로봇이 추적 모드일 경우에,opencv를 활용하여, 바운딩박스를 통해, 바운딩박스가 카메라 프레임의 중심에 위치하게끔 제어를 하도록 하였다.다만 카메라 프레임 정중앙에 위치하지 않고, 의도치 않은 회전문제를 겪었다.
  <br> - 해결 : 해당 문제 이전에는 코드상 회전 값이 너무 커서 디텍팅하고 나서 회전하면, 광물이 화면 밖으로 나가버리는 문제가 있었는데 이는 회전 값을 적게 주어 해결하였다.
  <br> -  또한 치우치는 문제는 회전 방향에 대한 이해를 통해, 부호를 수정하여 해결 되었다|더미|카메라 인식 알고리즘 구현, 구동제어|
|최민호|더미|더미|로봇 카메라 제어|
|류승기|더미|더미|로봇 상호작용 알고리즘 구현|
|정승연|더미|더미|맵 & 객체 제작, 테스트 GUI 기능 및 프로토타입 생성 & 구현|
