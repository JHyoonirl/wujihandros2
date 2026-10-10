# 로컬 통합 메모

이 디렉터리는 공식 `wuji-technology/wujihandros2` 저장소와 공식
`external/wuji-description` 서브모듈을 그대로 체크아웃한 경로입니다. WUJI
드라이버의 기준은 upstream 저장소의 README와 소스이며, 이 문서는 해당
드라이버를 `Hand_retargetting`에서 사용하는 방법만 기록합니다.

## 공식 저장소 업데이트

```bash
bash tools/update_wuji_driver.sh
```

이 스크립트는 기본적으로 체크아웃을 `v1.1.0`에 고정합니다. 공식 릴리스의
다른 버전을 의도적으로 시험할 때만 다음처럼 태그를 지정하십시오.

```bash
WUJI_DRIVER_TAG=<tag> bash tools/update_wuji_driver.sh
```

이 작업공간에 별도의 손 드라이버를 복사하지 마십시오. 장갑 시리얼 번호,
펌웨어 버전, 현장별 보정 메모리는 `hardware/wuji/` 아래에 보관하십시오.

## SDK 사전 조건

공식 ROS 2 드라이버는 `wujihandcpp`를 링크합니다. Ubuntu 24.04에서 직접
빌드하려면 프로젝트가 고정한 공식 amd64 패키지를 먼저 설치하십시오.

```bash
bash tools/install_wuji_sdk.sh
```

설치 파일은 작업공간의 `downloads/wuji/`에 캐시되고 SHA-256 체크섬을
검증합니다. 최초 설치에는 sudo 권한이 필요합니다. Docker를 사용하면
Dockerfile이 같은 버전을 이미지 안에 설치하므로 호스트 설치는 생략할 수
있습니다.

프로젝트 Dockerfile은 이 패키지를 자동으로 설치합니다. 호스트에 SDK가
없으면 `tools/build_ros.sh`가 MANUS 패키지만 빌드하고 WUJI 패키지는 경고와
함께 건너뜁니다.

## 공식 손 드라이버 실행

```bash
source /opt/ros/jazzy/setup.bash
bash tools/build_ros.sh -DBUILD_TESTING=OFF
source ros2_ws/install/setup.bash

# 손이 두 개 이상 연결되어 있으면 hand_side로 사용할 쪽을 선택합니다.
ros2 launch wujihand_bringup wujihand.launch.py \
  hand_name:=hand_0 hand_side:=left
```

`tools/build_ros.sh`는 MANUS와 WUJI 패키지를 각각
`colcon build --symlink-install`로 빌드합니다. 따라서 Python 실행 스크립트는
설치 공간에 복사하지 않고 소스 경로를 직접 가리킵니다. `CMakeLists.txt`나
패키지 의존성을 바꾼 경우에는 여전히 다시 빌드해야 합니다.

`wuji_description`에서 upstream 패키지 버전 형식 경고가 한 번 출력될 수
있지만, 빌드를 실패시키는 오류는 아닙니다.

공식 드라이버는 자체 저역통과 실시간 컨트롤러를 사용합니다. 명령 및 상태
인터페이스는 다음과 같습니다.

```text
/hand_0/joint_states     sensor_msgs/msg/JointState, 20개 위치값
/hand_0/joint_commands   sensor_msgs/msg/JointState, 20개 명령값
/hand_0/set_enabled      wujihand_msgs/srv/SetEnabled
```

## 엄지를 제외한 네 손가락 개폐 시험

`four_finger_cycle.py`는 검증용 위치 궤적입니다. 기본값은 **dry run**이라서
하드웨어에 명령을 보내지 않습니다. 엄지의 4개 관절은 현재값에 고정하고,
검지부터 소지까지는 프로젝트의 기존 동작 패턴인 `joint0`, `joint2`,
`joint3`만 현재값에서 최대 `0.12 rad` 움직입니다. 각 손가락의 `joint1`은
고정합니다.

먼저 드라이버를 실행한 터미널과 별도의 터미널에서 명령 배열을 검증합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 run wujihand_bringup four_finger_cycle.py --ros-args \
  -p dry_run:=true
```

손 주변을 비운 뒤 실제 시험을 명시적으로 실행합니다. 드라이버가 먼저
실행 중이어야 하며, `hand_name`은 드라이버의 네임스페이스와 같아야 합니다.

```bash
ros2 run wujihand_bringup four_finger_cycle.py --ros-args \
  -p hand_name:=hand_0 \
  -p dry_run:=false \
  -p amplitude:=0.12 \
  -p move_duration:=2.5 \
  -p cycles:=1 \
  -p disable_on_exit:=true
```

`amplitude`는 안전을 위해 `0.25 rad` 이하만 허용합니다. 반대 방향으로
움직여야 하는 장치에서는 `-p direction:=-1.0`을 사용할 수 있습니다. 사이클이
끝나면 기준 자세를 다시 전송한 뒤 `/hand_0/set_enabled`에 전체 관절 비활성화
요청을 보내고 종료합니다. 다음 실행 때는 기본적으로 전체 관절을 먼저
`enabled=true`로 전환하므로, 이전 실행이 모터를 비활성화했어도 별도 enable
명령이 필요하지 않습니다. 실행 중 `Ctrl+C`로 중단해도 기준 자세를 보낸 뒤
같은 비활성화 요청을 수행합니다. 반복해서 다른 명령을 보내려면
`-p disable_on_exit:=false`로 종료 시 비활성화를 끌 수 있습니다. 서비스가
응답하지 않으면 모터 상태를 확인하는 오류를 출력합니다.

공식적으로 지원되는 안전한 정지 절차는 home 스크립트로 현재 자세를 0으로
이동시킨 다음, 공식 서비스로 모든 관절을 비활성화하는 것입니다.

```bash
ros2 run wujihand_bringup home.py --ros-args \
  -p hand_name:=hand_0 -p duration:=1.5 -p rate:=120.0
ros2 service call /hand_0/set_enabled wujihand_msgs/srv/SetEnabled \
  "{finger_id: 255, joint_id: 255, enabled: false}"
```

서비스가 `success: true`를 반환해도 드라이버 프로세스가 실행 중이면
realtime PDO 컨트롤러가 계속 연결되어 있을 수 있습니다. 힘을 확실히
해제하려면 서비스 호출 직후 launch 터미널에서 `Ctrl+C`로 드라이버를
종료하십시오. 공식 SDK의 종료 과정이 PDO를 끄고 비활성화된 관절 상태를
유지합니다.

여기서 `set_enabled(false)`는 모터를 비활성화하는 기능이지, 사람이 손가락을
자유롭게 역구동하는 relax/compliance 모드가 아닙니다. WUJI Hand 1은 공식
문서에서 구동 방식이 self-locking rotary joint, 제어 방식이 position control로
정의되어 있고, 이 ROS 2 드라이버에도 임피던스·토크·backdrive 모드가
노출되어 있지 않습니다. 따라서 모터가 꺼져도 기계적으로 손이 쉽게
구부러지지 않을 수 있습니다. `ControlMode`를 임의의 값으로 쓰거나
`effort_limit`을 0으로 설정하는 방식은 공식 relax 절차로 확인되지 않았으므로
사용하지 마십시오.

이 순서는 MANUS 자동 시작 과정에는 넣지 않았습니다. MANUS와 WUJI는 같은
컨테이너의 USB 버스를 함께 사용할 수 있지만, `wujihand_driver`가 손을
제어하는 동안 WUJI Studio나 다른 WUJI SDK 프로세스를 동시에 실행하면 안
됩니다.

upstream 드라이버가 안내하는 지원 배포판은 Humble과 Kilted입니다. 이
작업공간은 Jazzy를 사용하므로, 모터 비활성화 서비스를 사용하기 전에 대상
호스트에서 빌드 결과와 실제 하드웨어 동작을 확인하십시오.
