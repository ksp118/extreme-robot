# AGENTS.md

이 파일은 Codex가 이 저장소에서 작업할 때 따르는 지침이다.

## 언어

- 사용자 응답은 항상 한국어로 한다.
- 문서, 커밋 메시지, PR 설명도 한국어를 기본으로 작성한다.

## 저장소 개요

- ROS 2 Humble 기반 극한로봇 로봇팔 워크스페이스다.
- 호스트에서는 주로 `git`과 `docker compose`만 사용한다.
- ROS 2 빌드, launch, run, topic 명령은 Docker 컨테이너 `ros2_humble` 안에서 실행한다.
- 호스트의 `./ros2_ws`는 컨테이너의 `/root/ros2_ws`에 bind mount된다.

## 기본 명령

호스트:

```bash
docker compose up -d
docker exec -it ros2_humble bash
```

컨테이너 내부:

```bash
cd /root/ros2_ws
colcon build
source install/setup.bash
```

패키지 단위 빌드:

```bash
colcon build --packages-select <package_name>
```

## 주요 패키지

- `dynamixel_control`: Dynamixel 제어, 텔레옵, YOLO 레거시 추적, MoveIt/FSM 브릿지 노드.
- `robot_arm_description`: URDF, 메시, RViz/TF 관련 launch.
- `robot_arm_moveit_config`: MoveIt 설정. mock 경로와 실제 서보 경로를 혼동하지 않는다.
- `robot_arm_msgs`: 팔/파워트레인 공용 메시지.
- `robot_arm_perception`: YOLO segmentation + depth 기반 인식 노드.
- `pick_test_pkg`: 그리퍼 단독 테스트 노드.

## 현재 브랜치 범위

- `feat/teleop-keyboard` 브랜치는 원격 조종 기능 구현용이다.
- 이 브랜치에서 기본 작업 범위는 `dynamixel_control`의 텔레옵 노드, 텔레옵 launch/RViz 설정, 그리고 텔레옵 확인에 필요한 `robot_arm_description`이다.
- FSM, 자율 픽 시퀀스, `arm_fsm_node.py`, MoveIt 실행 브릿지(`moveit_dynamixel_bridge.py`)는 사용자가 명시적으로 요청하지 않으면 수정하지 않는다.

## 하드웨어 주의사항

- 실제 Dynamixel 노드는 `/dev/ttyUSB0`와 실서보 버스가 필요하다.
- 같은 Dynamixel 버스를 두 노드가 동시에 잡지 않도록 한다.
- `dynamixel_position_node`와 `moveit_dynamixel_bridge`는 동시에 실행하지 않는다.
- 하드웨어 없이 RViz 텔레옵을 확인할 때는 `teleop_core`의 `publish_sim_joint_states` 경로를 사용한다.

## 5축 URDF 기준

- 텔레옵 기준 활성 조인트 이름은 `joint_1`부터 `joint_5`까지다.
- 기본 Dynamixel ID 매핑은 `joint_1..joint_5 -> 0..4`다.
- 기본 텔레옵 회전 제한은 `joint_1=[-pi,pi]`, `joint_2=[-pi,0]`, `joint_3=[0,pi]`, `joint_4=[-pi/2,pi/2]`, `joint_5=[-pi,pi]`다.
- 모터 center와 direction은 실하드웨어 캘리브레이션 전까지 기본값으로 둔다.

## 작업 원칙

- 변경 전에 관련 파일을 먼저 읽고 기존 패턴을 따른다.
- 불필요한 리팩터링과 범위 밖 파일 변경을 피한다.
- 빌드 산출물인 `ros2_ws/build`, `ros2_ws/install`, `ros2_ws/log`는 커밋 대상이 아니다.
- 의존성 추가가 필요하면 임시 설치보다 `Dockerfile` 반영을 우선한다.
