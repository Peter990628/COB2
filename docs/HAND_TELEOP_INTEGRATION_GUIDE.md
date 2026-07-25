# Hand Teleop 통합 및 실행 가이드

이 문서는 `hand_teleop` 패키지를 다른 개발자의 Ubuntu 22.04 / ROS 2
Humble 환경으로 옮겨서 실행하거나, 다른 호텔 안내 로봇 노드와 통합하기 위한
인수인계 문서이다.

현재 기준 대상 장비는 다음과 같다.

- OS: Ubuntu 22.04
- ROS 2: Humble
- Robot: Doosan Robotics M0609
- Robot namespace: `dsr01`
- Gripper: OnRobot RG2, Modbus TCP
- Hand camera: 일반 2D USB 웹캠
- Python environment: `venv --system-site-packages`

> **중요:** `hand_tracker_node_ver_depth`라는 이름에 `depth`가 들어가지만,
> RealSense 깊이 영상을 사용하지 않는다. 일반 2D 웹캠에서 보이는 손바닥
> 크기 변화를 이용해 카메라에 가까워졌는지/멀어졌는지를 상대값으로 추정한다.
> `point.z`는 mm 단위 실제 거리가 아니다.

---

## 1. 패키지 구성과 실행 범위

실제로 통합할 노드는 다음 세 개다.

| 실행 이름 | 역할 | 실제 로봇 이동 |
|---|---|---|
| `hand_tracker_node` | 2D 손 X/Y와 주먹 상태 발행 | 안 함 |
| `hand_tracker_node_ver_depth` | 손 X/Y, 상대 깊이 Z, 주먹 상태 발행 | 안 함 |
| `hand_follow_robot_node_ver_depth` | 위 토픽을 받아 M0609과 RG2 제어 | `dry_run:=false`일 때 함 |

프로젝트에서 주로 사용하는 조합은 다음과 같다.

```text
USB 웹캠
  → hand_tracker_node_ver_depth
  → /hand_teleop/* 토픽
  → hand_follow_robot_node_ver_depth
  → Doosan M0609 + RG2
```

`hand_follow_robot_node_ver_depth`는 `/hand_teleop/depth_valid`가 필요하므로
기본 `hand_tracker_node`와 조합하지 않는다. 반드시
`hand_tracker_node_ver_depth`와 조합한다.

두 tracker는 같은 카메라와 같은 X/Y/fist 토픽을 사용하므로 동시에 실행하지
않는다.

다른 시스템이 손 인식 결과만 사용할 경우에는
`hand_tracker_node_ver_depth`만 실행하고 `/hand_teleop/*` 토픽을 구독하면
된다. 로봇 제어 노드를 반드시 함께 사용할 필요는 없다.

현재 패키지에는 launch 파일이 없다. 통합 담당자는 노드를 각각 실행하거나
팀의 상위 launch 파일에 별도 ROS 프로세스로 등록해야 한다.

### 통합 대상이 아닌 파일

다음 파일들은 다른 사람의 코드를 비교하기 위해 보관한 참고 예제이며 현재
통합 대상이 아니다.

```text
src/hand_teleop/test/jog_tracking.py
src/hand_teleop/test/tracking.py
```

이 파일들은 단위 테스트가 아니고 내부에서 실제 `movej`, RG2 명령,
`servol_stream` 발행을 수행한다. 직접 실행하거나 `test_*.py`로 이름을
바꾸지 않는다.

---

## 2. 다른 PC로 전달할 파일

다음 패키지 디렉터리 전체를 전달한다.

```text
hand_teleop/
├── hand_teleop/
│   ├── __init__.py
│   ├── hand_tracker_node.py
│   ├── hand_tracker_node_ver_depth.py
│   ├── hand_follow_robot_node_ver_depth.py
│   └── onrobot.py
├── models/
│   └── hand_landmarker.task
├── resource/
├── test/
├── package.xml
├── requirements.txt
├── setup.cfg
└── setup.py
```

반드시 포함해야 하는 파일은 다음과 같다.

```text
models/hand_landmarker.task
```

다음 항목은 복사하지 말고 새 PC에서 다시 만든다.

```text
.venv_hand/
build/
install/
log/
__pycache__/
.pytest_cache/
```

가상환경에는 절대경로가 저장될 수 있으므로 다른 컴퓨터에 그대로 복사하면
안 된다.

로봇 제어까지 사용할 컴퓨터에는 Doosan ROS 2 패키지도 같은 workspace의
`src` 아래에 있어야 한다.

```text
~/cobot_ws/src/doosan-robot2
~/cobot_ws/src/hand_teleop
```

현재 사용 중인 Doosan 저장소는 다음과 같다.

```text
https://github.com/ROKEY-SPARK/doosan-robot2_2026
```

손 추적 노드 자체는 Doosan API를 import하지 않는다. 따라서 손 인식 토픽만
발행하는 전용 PC라면 카메라 노드만 실행할 수 있다. 다만 현재
`package.xml`에는 전체 패키지의 실행 의존성으로 `dsr_msgs2`,
`dsr_common2`가 선언되어 있다. 표준 `rosdep`과 전체 패키지 검증까지 같은
조건으로 수행하려면 동일한 Doosan 패키지를 workspace에 함께 두는 것이
가장 간단하다.

---

## 3. 운영체제와 ROS 준비

ROS 2 Humble이 설치되어 있다는 전제에서 필요한 기본 도구를 설치한다.

```bash
sudo apt update
sudo apt install -y \
  python3-venv \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep \
  v4l-utils
```

ROS 환경을 확인한다.

```bash
source /opt/ros/humble/setup.bash
echo "$ROS_DISTRO"
```

정상 결과:

```text
humble
```

`rosdep`이 처음인 컴퓨터에서만 다음 명령을 실행한다.

```bash
sudo rosdep init
rosdep update
```

이미 초기화된 컴퓨터에서 `rosdep init`을 다시 실행하면 이미 존재한다는
메시지가 나올 수 있다. 그 경우 `rosdep update`만 실행한다.

workspace 의존성은 다음처럼 설치할 수 있다.

```bash
cd ~/cobot_ws

rosdep install \
  --from-paths src \
  --ignore-src \
  -r \
  -y \
  --skip-keys common2
```

---

## 4. Python 가상환경 만들기

팀원 컴퓨터에서도 별도의 가상환경을 만드는 것을 권장한다. MediaPipe,
OpenCV, NumPy 버전 충돌을 피하고 동일한 실행 환경을 재현하기 위해서다.

### 4.1 새 가상환경 생성

```bash
cd ~/cobot_ws

python3 -m venv \
  --system-site-packages \
  .venv_hand
```

`--system-site-packages`가 필요한 이유는 `/opt/ros/humble`에 설치된
`rclpy`와 ROS 메시지 패키지를 가상환경에서도 사용하기 위해서다.

### 4.2 가상환경 활성화

```bash
source /opt/ros/humble/setup.bash
source ~/cobot_ws/.venv_hand/bin/activate
export PYTHONNOUSERSITE=1
```

`PYTHONNOUSERSITE=1`은 `~/.local/lib/python3.10/site-packages`에 설치된
다른 OpenCV, NumPy, Matplotlib이 가상환경 안으로 섞이는 것을 막는다.

이 값은 다음 작업을 할 때 모두 설정하는 것이 좋다.

- `pip install`
- `colcon build`
- `colcon test`
- `ros2 run`

### 4.3 Python 패키지 설치

```bash
cd ~/cobot_ws

python -m pip install --upgrade pip
python -m pip install \
  -r src/hand_teleop/requirements.txt
```

현재 고정 버전은 다음과 같다.

```text
mediapipe==0.10.35
numpy==2.2.6
opencv-contrib-python==5.0.0.93
matplotlib==3.10.9
pymodbus==2.5.3
```

별도로 `opencv-python`을 추가 설치하지 않는다. 새 가상환경에
`requirements.txt`만 설치하는 것이 가장 안전하다.

### 4.4 설치 확인

```bash
which python
python -m pip -V

python -c "
import cv2
import mediapipe
import numpy
import pymodbus
import rclpy
print('cv2       =', cv2.__version__)
print('mediapipe =', mediapipe.__version__)
print('numpy     =', numpy.__version__)
print('pymodbus  =', pymodbus.__version__)
print('rclpy import OK')
"
```

`which python`은 다음 가상환경 Python을 가리켜야 한다.

```text
~/cobot_ws/.venv_hand/bin/python
```

---

## 5. MediaPipe 모델 확인

다음 파일이 있는지 확인한다.

```bash
ls -lh \
  ~/cobot_ws/src/hand_teleop/models/hand_landmarker.task
```

파일이 없다면 공식 모델을 내려받는다.

```bash
cd ~/cobot_ws/src/hand_teleop
mkdir -p models

wget -O models/hand_landmarker.task \
  https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task
```

모델을 새로 추가한 후에는 패키지를 다시 빌드해야 설치 공간의
`share/hand_teleop/models`에도 반영된다.

---

## 6. 패키지 빌드

새 컴퓨터에서 Doosan 패키지까지 함께 처음 빌드할 때:

```bash
cd ~/cobot_ws

source /opt/ros/humble/setup.bash
source ~/cobot_ws/.venv_hand/bin/activate
export PYTHONNOUSERSITE=1

python -m colcon build \
  --packages-up-to hand_teleop \
  --symlink-install

source ~/cobot_ws/install/setup.bash
```

Doosan 패키지가 이미 빌드된 workspace에서 `hand_teleop`만 다시 빌드할 때:

```bash
cd ~/cobot_ws

python -m colcon build \
  --packages-select hand_teleop \
  --symlink-install

source ~/cobot_ws/install/setup.bash
```

실행 파일 등록을 확인한다.

```bash
ros2 pkg executables hand_teleop
```

정상적으로 다음 세 실행 이름이 보여야 한다.

```text
hand_teleop hand_tracker_node
hand_teleop hand_tracker_node_ver_depth
hand_teleop hand_follow_robot_node_ver_depth
```

---

## 7. 새 터미널을 열 때마다 실행할 환경 설정

각 터미널에서 다음 순서를 지킨다.

```bash
cd ~/cobot_ws
source /opt/ros/humble/setup.bash
source ~/cobot_ws/.venv_hand/bin/activate
export PYTHONNOUSERSITE=1
source ~/cobot_ws/install/setup.bash
```

프롬프트 앞에 다음 표시가 나오면 가상환경이 활성화된 것이다.

```text
(.venv_hand)
```

빌드 시 생성되는 ROS 실행 스크립트에는 빌드할 때 사용한 Python의 절대경로가
들어간다. `.venv_hand`를 다른 경로로 옮기거나 삭제했다면 새 위치의 venv를
활성화한 상태에서 `hand_teleop`을 다시 빌드한다.

---

## 8. 웹캠 확인

사용할 카메라 목록을 확인한다.

```bash
v4l2-ctl --list-devices
ls -l /dev/video*
```

기본 카메라 인덱스는 `0`이다. 다른 카메라를 사용한다면 실행할 때
`camera_index`를 바꾼다.

```bash
ros2 run hand_teleop hand_tracker_node_ver_depth \
  --ros-args \
  -p camera_index:=2
```

카메라 권한 오류가 있으면 현재 사용자가 `video` 그룹에 속해 있는지
확인한다.

```bash
groups
```

---

## 9. 손 추적 노드만 실행하기

로봇 bringup 없이도 실행할 수 있다.

```bash
ros2 run hand_teleop hand_tracker_node_ver_depth
```

노드를 시작하면 편 손을 평소 조작할 거리에서 약 45프레임 동안 유지한다.
손을 앞뒤로 움직이거나 주먹을 쥐지 않는다.

정상적으로 보정되면 다음과 비슷한 로그가 나온다.

```text
상대 깊이 중립 보정 완료
```

디버그 창의 주요 표시는 다음과 같다.

- `published x/y`: 필터링된 화면상 손 중심 좌표
- `depth raw`: 손 크기에서 직접 계산한 상대 깊이
- `depth filtered`: EMA 필터가 적용된 상대 깊이
- `depth_valid`: 중립 보정 완료 및 현재 깊이 사용 가능 여부
- `raw_fist`: 현재 한 프레임의 주먹 판정
- `stable fist`: 여러 프레임으로 확정한 주먹 상태

`Q`, `q` 또는 `ESC`로 종료한다.

GUI가 필요 없는 통합 환경에서는 다음처럼 실행한다.

```bash
ros2 run hand_teleop hand_tracker_node_ver_depth \
  --ros-args \
  -p show_debug_window:=false
```

해상도와 처리 속도를 지정하는 예:

```bash
ros2 run hand_teleop hand_tracker_node_ver_depth \
  --ros-args \
  -p camera_index:=0 \
  -p frame_width:=640 \
  -p frame_height:=480 \
  -p camera_fps:=30.0 \
  -p processing_rate_hz:=30.0
```

---

## 10. ROS 토픽 인터페이스

`hand_tracker_node_ver_depth`는 다음 토픽을 발행한다.

| 토픽 | 메시지 타입 | 의미 |
|---|---|---|
| `/hand_teleop/hand_position` | `geometry_msgs/msg/PointStamped` | 정규화 X/Y와 상대 깊이 Z |
| `/hand_teleop/hand_detected` | `std_msgs/msg/Bool` | 현재 프레임 손 검출 여부 |
| `/hand_teleop/depth_valid` | `std_msgs/msg/Bool` | 상대 깊이 보정 및 사용 가능 여부 |
| `/hand_teleop/fist` | `std_msgs/msg/Bool` | 여러 프레임으로 안정화된 주먹 여부 |
| `/hand_teleop/annotated_image` | `sensor_msgs/msg/Image` | 랜드마크와 상태가 그려진 UI용 BGR 영상 |

### `hand_position` 좌표 규칙

```text
point.x: 화면 왼쪽 0.0 → 화면 오른쪽 1.0
point.y: 화면 위쪽 0.0 → 화면 아래쪽 1.0
point.z: 카메라에서 멀어짐 -1.0 → 카메라에 가까워짐 +1.0
```

`header.frame_id`:

```text
webcam_normalized_relative_depth
```

`point.z`는 실제 거리 mm가 아니다.

### UI용 주석 영상

`/hand_teleop/annotated_image`는 웹캠 프레임에 다음 정보를 모두 그린 최종
영상이다.

- 손 랜드마크와 연결선
- 필터 적용 전/후 손바닥 중심
- 손 검출 및 주먹 판정 상태
- 상대 깊이 값, 손바닥 크기, FAR/NEAR 막대

메시지 형식은 `sensor_msgs/msg/Image`, 인코딩은 `bgr8`이다. 운영체제의
창 제목줄이나 OpenCV 창 테두리는 포함하지 않으며, UI에 필요한 실제 영상
영역만 발행한다.

관련 파라미터:

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `publish_annotated_image` | `true` | UI용 영상 발행 여부 |
| `annotated_image_topic` | `/hand_teleop/annotated_image` | 영상 토픽 이름 |
| `annotated_image_frame_id` | `webcam_annotated` | 메시지 헤더 frame ID |
| `show_debug_window` | `true` | 로컬 OpenCV 창 표시 여부 |

UI만 사용할 때는 로컬 창을 꺼도 영상 토픽은 계속 발행된다.

```bash
ros2 run hand_teleop hand_tracker_node_ver_depth \
  --ros-args \
  -p show_debug_window:=false \
  -p publish_annotated_image:=true
```

영상 토픽 확인:

```bash
ros2 topic info /hand_teleop/annotated_image
ros2 topic hz /hand_teleop/annotated_image
```

`rqt_image_view`가 설치되어 있다면 다음 명령으로 실행한 뒤
`/hand_teleop/annotated_image`를 선택할 수 있다.

```bash
ros2 run rqt_image_view rqt_image_view
```

기본 640×480 BGR8 영상을 30 Hz로 발행하면 DDS 오버헤드를 제외해도 약
27.6 MB/s이므로, UI에서 영상을 사용하지 않는 실행에서는 다음처럼 끌 수 있다.

```bash
ros2 run hand_teleop hand_tracker_node_ver_depth \
  --ros-args \
  -p publish_annotated_image:=false
```

### 손이 보이지 않을 때

손이 사라지면:

```text
/hand_teleop/hand_detected = false
/hand_teleop/depth_valid   = false
/hand_teleop/fist          = false
```

가 발행된다. 잘못된 `(0, 0, 0)` 위치는 새로 발행하지 않는다.

따라서 통합 노드는 마지막 `hand_position`만 계속 사용하면 안 된다.
반드시 다음 조건을 모두 확인해야 한다.

```text
hand_detected == true
depth_valid == true
마지막 hand_position이 timeout보다 최근 값
```

현재 로봇 제어 노드의 기본 timeout은 0.30초다.

### 토픽 확인 명령

```bash
ros2 topic list | grep hand_teleop

ros2 topic echo \
  /hand_teleop/hand_position

ros2 topic echo \
  /hand_teleop/hand_detected

ros2 topic echo \
  /hand_teleop/depth_valid

ros2 topic echo \
  /hand_teleop/fist

ros2 topic info \
  /hand_teleop/annotated_image
```

발행 주기 확인:

```bash
ros2 topic hz \
  /hand_teleop/hand_position
```

---

## 11. 다른 ROS 노드에서 토픽을 사용할 때

통합 노드의 권장 처리 흐름:

```text
hand_detected 수신
  ├─ false → 로봇 목표 갱신 중지
  └─ true
       ↓
depth_valid 확인
  ├─ false → X/Y만 사용하거나 전체 이동 대기
  └─ true
       ↓
PointStamped timestamp 확인
  ├─ 오래됨 → 정지 처리
  └─ 최신 → 좌표 변환 및 안전 제한 적용
```

권장 사항:

- 영상 좌표를 로봇 mm 좌표로 바로 사용하지 않는다.
- 로봇의 작업범위로 한 번 더 clamp한다.
- 마지막 메시지 수신 시간을 이용한 watchdog을 둔다.
- 손 재검출 직후 첫 좌표로 큰 이동을 만들지 않는다.
- 속도, 가속도, 한 주기 이동거리 제한을 둔다.
- 주먹 토픽은 그리퍼 명령이 성공했다는 뜻이 아니라 사용자의 손 모양이다.
- 그리퍼 명령은 별도 상태 머신과 통신 오류 처리를 둔다.

### 최소 Python 구독 예시

아래 코드는 토픽 연결과 timeout 처리 구조만 보여주는 예시이며 로봇을
움직이지 않는다.

```python
#!/usr/bin/env python3

import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import Bool


class HandInputConsumer(Node):
    def __init__(self):
        super().__init__("hand_input_consumer")

        self.hand_detected = False
        self.depth_valid = False
        self.fist = False
        self.latest_position = None
        self.last_position_received_at = None
        self.timeout_sec = 0.30

        self.create_subscription(
            PointStamped,
            "/hand_teleop/hand_position",
            self.position_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/hand_teleop/hand_detected",
            self.detected_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/hand_teleop/depth_valid",
            self.depth_valid_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/hand_teleop/fist",
            self.fist_callback,
            10,
        )

        self.create_timer(0.05, self.process_latest_input)

    def position_callback(self, message):
        self.latest_position = (
            float(message.point.x),
            float(message.point.y),
            float(message.point.z),
        )
        self.last_position_received_at = time.monotonic()

    def detected_callback(self, message):
        self.hand_detected = bool(message.data)

    def depth_valid_callback(self, message):
        self.depth_valid = bool(message.data)

    def fist_callback(self, message):
        self.fist = bool(message.data)

    def process_latest_input(self):
        if not self.hand_detected:
            return
        if not self.depth_valid:
            return
        if (
            self.latest_position is None
            or self.last_position_received_at is None
        ):
            return

        age = time.monotonic() - self.last_position_received_at
        if age > self.timeout_sec:
            return

        hand_x, hand_y, hand_z = self.latest_position
        self.get_logger().info(
            f"valid hand: x={hand_x:.3f}, "
            f"y={hand_y:.3f}, z={hand_z:+.3f}, "
            f"fist={self.fist}"
        )


def main():
    rclpy.init()
    node = HandInputConsumer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

통합 시스템이 X/Y만 사용하고 상대 깊이는 사용하지 않는다면
`depth_valid`를 필수 조건에서 제외할 수 있다. 하지만 그 경우 Z축을 이용한
로봇 BASE X 이동도 비활성화해야 한다.

### 로봇 명령 소유권

팀의 통합 노드가 직접 M0609를 제어한다면
`hand_follow_robot_node_ver_depth`를 동시에 실행하지 않는다.

```text
방법 A: 기존 hand_follow_robot_node_ver_depth가 로봇 제어
방법 B: 팀 통합 노드가 손 토픽을 구독하고 로봇 제어
```

두 방법 중 하나만 선택한다. 두 노드가 동시에 `servol`, `speedl`,
`movej` 계열 명령을 보내면 명령 충돌, 모션 거부, 급정지 또는 fault가 발생할
수 있다.

팀 launch 파일에 Doosan bringup과 follower를 함께 넣는다면 follower를
무조건 동시에 시작하지 않는다. `/dsr01` controller와 필요한 서비스가
준비된 뒤 follower가 시작되도록 순서 또는 준비 확인 절차를 둔다.

현재 publisher와 subscriber 수는 다음 명령으로 확인할 수 있다.

```bash
ros2 topic info \
  /hand_teleop/hand_position \
  --verbose
```

현재 로봇 제어 노드의 기본 축 변환은 다음과 같다.

```text
손 화면 +X(오른쪽) → 로봇 BASE +Y
손 화면 +Y(아래쪽) → 로봇 BASE -Z
손 상대 깊이 +Z    → 기본값에서 로봇 BASE -X
```

로봇 BASE X 방향은 설치 방향에 따라
`x_direction_sign`으로 변경할 수 있다.

---

## 12. 로봇 없이 전체 연결 Dry run

Dry run에서는 로봇 초기 이동, `servol`, RG2 명령을 보내지 않는다.
Doosan 서비스가 실행 중이지 않아도 손 토픽과 변환 로그를 확인할 수 있다.

터미널 1:

```bash
ros2 run hand_teleop hand_tracker_node_ver_depth
```

터미널 2:

```bash
ros2 run hand_teleop hand_follow_robot_node_ver_depth \
  --ros-args \
  -p dry_run:=true \
  -p gripper.enabled:=false
```

로그에 다음 값이 보이는지 확인한다.

```text
hand=(x, y, z)
offset=(x_mm, y_mm, z_mm)
state=tracking
```

Dry run 검증 전에는 실로봇 모드로 전환하지 않는다.

---

## 13. Doosan M0609 bringup

### 가상 모드/RViz

```bash
ros2 launch dsr_bringup2 \
  dsr_bringup2_rviz.launch.py \
  mode:=virtual \
  model:=m0609 \
  name:=dsr01
```

### 실제 로봇

`<ROBOT_IP>`를 실제 컨트롤러 IP로 바꾼다.

```bash
ros2 launch dsr_bringup2 \
  dsr_bringup2_rviz.launch.py \
  mode:=real \
  model:=m0609 \
  name:=dsr01 \
  host:=<ROBOT_IP>
```

`model:=m0609`를 반드시 명시한다. Doosan launch 파일의 기본 모델이 다른
모델일 수 있다.

현재 follower의 다음 값은 코드에 고정되어 있다.

```text
ROBOT_ID    = dsr01
ROBOT_MODEL = m0609
```

따라서 bringup namespace도 `dsr01`이어야 한다. 다른 robot ID나 model을
사용하려면 현재는 실행 파라미터가 아니라 코드 수정이 필요하다.

bringup 후 확인:

```bash
ros2 node list | grep dsr01
ros2 service list | grep /dsr01
```

---

## 14. 실제 로봇 저범위 1차 시험

> 실제 로봇 시험 전 주변 사람과 장애물을 치우고 비상정지 장치를 잡을 수 있는
> 위치에서 시작한다. 아래 수치도 충돌 회피를 보장하지 않는다.

처음에는 RG2를 끄고 이동 범위를 작게 제한한다.

터미널 1:

```bash
ros2 launch dsr_bringup2 \
  dsr_bringup2_rviz.launch.py \
  mode:=real \
  model:=m0609 \
  name:=dsr01 \
  host:=<ROBOT_IP>
```

터미널 2:

```bash
ros2 run hand_teleop hand_tracker_node_ver_depth
```

터미널 3:

```bash
ros2 run hand_teleop hand_follow_robot_node_ver_depth \
  --ros-args \
  -p dry_run:=false \
  -p gripper.enabled:=false \
  -p initial_joint_velocity:=5.0 \
  -p initial_joint_acceleration:=10.0 \
  -p x_travel_max_mm:=50.0 \
  -p y_offset_min_mm:=-50.0 \
  -p y_offset_max_mm:=50.0 \
  -p z_offset_min_mm:=-50.0 \
  -p z_offset_max_mm:=50.0 \
  -p maximum_step_mm:=0.5 \
  -p command_linear_acceleration_mm_s2:=10.0
```

현재 초기 관절 자세:

```text
[-90.0, 0.0, 90.0, -90.0, 90.0, 90.0]
```

실행하면 먼저 이 관절 자세로 이동한 뒤 도착한 TCP pose를 추종 원점으로
저장한다.

`move_to_initial_pose:=false`를 사용하면 현재 TCP pose를 원점으로
사용하지만, 자동 홈 복귀 경로와 원점 동작을 별도로 확인해야 한다.

---

## 15. RG2 그리퍼 사용

로봇 이동을 충분히 검증한 후에만 RG2를 활성화한다.

연결 확인:

```bash
ping -c 3 192.168.1.1
```

기본 Modbus TCP 설정:

```text
IP:   192.168.1.1
Port: 502
```

RG2 드라이버의 유효 범위:

```text
폭:   0~110 mm
힘:   3~40 N
```

코드의 ROS 파라미터는 실제 단위인 mm와 N을 사용한다. 내부 드라이버가
Modbus raw 단위로 10배 변환한다.

```text
80.0 mm → raw 800
30.0 N  → raw 300
```

현재 코드의 실제 기본값은 다음과 같다.

```text
OPEN:  30.0 mm / 40.0 N
FIST:   1.0 mm / 40.0 N
```

원하는 제스처 설정과 다를 수 있으므로 통합 실행 시 값을 명시하는 것을
권장한다.

```bash
ros2 run hand_teleop hand_follow_robot_node_ver_depth \
  --ros-args \
  -p dry_run:=false \
  -p gripper.enabled:=true \
  -p gripper.ip:=192.168.1.1 \
  -p gripper.port:=502 \
  -p gripper.open_width_mm:=80.0 \
  -p gripper.open_force_n:=3.0 \
  -p gripper.closed_width_mm:=2.0 \
  -p gripper.closed_force_n:=30.0
```

손을 잃으면 현재 코드는 마지막 그리퍼 상태를 유지한다. 물체를 들고 있을 때
손이 사라졌다는 이유로 자동으로 그리퍼를 열지 않는다.

RG2 상태의 `grip_detected` 비트는 드라이버에서 읽을 수 있지만, 현재
`hand_follow_robot_node_ver_depth`는 파지 성공 판정에 사용하지 않는다.
현재 코드는 RG2 busy 해제와 safety fault를 중심으로 확인한다. 통합 시스템이
실제 물체 파지 성공 여부를 필요로 한다면 별도 판정 로직이 필요하다.

---

## 16. 주요 로봇 제어 파라미터

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `dry_run` | `true` | 실제 로봇/RG2 명령 차단 |
| `move_to_initial_pose` | `true` | 시작 시 초기 관절 자세로 이동 |
| `control_rate_hz` | `10.0` | 추종 제어 주기 |
| `hand_timeout_sec` | `0.30` | 손 위치 토픽 timeout |
| `maximum_step_mm` | `1.0` | 한 주기 XYZ 전체 이동거리 |
| `command_linear_acceleration_mm_s2` | `20.0` | 외부 명령 가속도 제한 |
| `servo_linear_velocity` | `30.0` | `servol` 선속도 한도 |
| `servo_linear_acceleration` | `60.0` | `servol` 내부 선가속도 |
| `servo_reach_time_sec` | `0.15` | 각 목표 도달 요청 시간 |
| `recovery.loss_grace_sec` | `0.5` | 손 재검출 유예시간 |
| `recovery.hold_before_home_sec` | `1.0` | 정지 후 홈 복귀 전 대기 |
| `recovery.home_velocity` | `5.0` | 자동 홈 관절속도 |
| `recovery.home_acceleration` | `10.0` | 자동 홈 관절가속도 |

현재 기본 설정에서 외부 목표 속도 상한:

```text
maximum_step_mm × control_rate_hz
= 1.0 mm × 10 Hz
= 약 10 mm/s
```

한 주기 속도 변화 상한:

```text
command_linear_acceleration_mm_s2 / control_rate_hz
= 20 mm/s² / 10 Hz
= 2 mm/s
```

`maximum_step_mm`을 크게 올리기 전에 실제 J5/J6 관절 속도와 특이점 접근
여부를 확인한다.

---

## 17. 손 유실 시 동작

정상적인 손 유실 시 로봇 제어 노드는 다음 순서로 동작한다.

```text
손 또는 깊이 입력 유실
→ speedl 목표 속도 0으로 감속
→ 실제 TCP 속도 0 확인
→ 현재 위치에서 잠시 유지
→ 손이 계속 보이지 않으면 initial_joint_pose로 저속 복귀
```

주의:

- 자동 홈 복귀는 MoveIt 충돌 회피 경로가 아니다.
- 현재 자세에서 초기 관절 자세까지 `amovej`한다.
- 전체 복귀 궤적을 실제 설치 환경에서 미리 저속 검증해야 한다.
- fault 또는 노드 종료 시에는 정상 홈 복귀 대신 soft stop을 요청한다.

---

## 18. 단위 테스트

스무딩 계산 테스트는 실제 로봇을 움직이지 않는다.

```bash
cd ~/cobot_ws

python -m colcon test \
  --packages-select hand_teleop \
  --event-handlers console_direct+

python -m colcon test-result --verbose
```

정상 결과:

```text
Summary: 10 tests, 0 errors, 0 failures, 0 skipped
```

테스트 파일 하나만 실행:

```bash
cd ~/cobot_ws/src/hand_teleop

python -m pytest -v \
  test/test_motion_smoothing.py
```

이 테스트는 목표 생성 수학만 검사한다. 실제 M0609의 관절 속도, 충돌,
통신 주기, 컨트롤러 fault까지 검증하는 시험은 아니다.

---

## 19. 여러 컴퓨터에서 ROS 토픽 공유

손 추적 노드와 통합 노드가 서로 다른 컴퓨터에서 실행된다면:

1. 두 컴퓨터 모두 ROS 2 Humble을 사용한다.
2. 두 컴퓨터의 `ROS_DOMAIN_ID`를 같게 설정한다.
3. 같은 네트워크에 연결한다.
4. 방화벽 또는 Wi-Fi AP가 DDS multicast를 차단하지 않는지 확인한다.
5. 양쪽에서 동일한 메시지 패키지를 사용할 수 있어야 한다.

예:

```bash
export ROS_DOMAIN_ID=30
```

양쪽 컴퓨터에서 같은 값을 사용한다.

송신 PC:

```bash
ros2 topic list | grep hand_teleop
```

수신 PC:

```bash
ros2 topic echo \
  /hand_teleop/hand_detected
```

토픽이 보이지 않으면 우선 `ROS_DOMAIN_ID`, 네트워크 multicast, 서로 다른
RMW 설정 여부를 확인한다.

---

## 20. 자주 발생하는 오류

### `No module named mediapipe`, `No module named cv2`

가상환경이 활성화되지 않았거나 requirements 설치가 빠진 경우다.

```bash
source ~/cobot_ws/.venv_hand/bin/activate
export PYTHONNOUSERSITE=1
python -m pip install \
  -r ~/cobot_ws/src/hand_teleop/requirements.txt
```

### NumPy/OpenCV/Matplotlib import 충돌

반드시 새 가상환경에서 고정 requirements를 사용하고 사용자 site-package를
차단한다.

```bash
source ~/cobot_ws/.venv_hand/bin/activate
export PYTHONNOUSERSITE=1
```

### `No module named rclpy`

ROS를 먼저 source하지 않았거나 `--system-site-packages` 없이 venv를 만든
경우다.

```bash
source /opt/ros/humble/setup.bash
```

그래도 실패하면 `.venv_hand`를 지우는 대신 원인을 확인한 후,
`--system-site-packages` 옵션으로 새 가상환경을 만든다.

### `dsr_msgs2` 또는 `DSR_ROBOT2` import 실패

Doosan ROS 2 패키지가 workspace에 없거나 빌드/source되지 않은 경우다.

```bash
cd ~/cobot_ws
source /opt/ros/humble/setup.bash
source ~/cobot_ws/.venv_hand/bin/activate
export PYTHONNOUSERSITE=1
source ~/cobot_ws/install/setup.bash
```

### `NoneType`에 `create_client`가 없다는 오류

`DSR_ROBOT2`를 ROS 노드 등록보다 먼저 import하면 발생할 수 있다.
현재 follower는 자신의 ROS 노드를 `DR_init`에 등록한 뒤
`DSR_ROBOT2`를 지연 import하도록 작성되어 있다. 통합하면서
`from DSR_ROBOT2 import ...`를 파일 최상단으로 옮기지 않는다.

### `ros2 run`에서 실행 파일을 찾지 못함

패키지를 다시 빌드하고 설치 공간을 source한다.

```bash
cd ~/cobot_ws

python -m colcon build \
  --packages-select hand_teleop \
  --symlink-install

source ~/cobot_ws/install/setup.bash
```

### `bad interpreter: .../.venv_hand/bin/python`

빌드 당시 사용한 venv가 삭제되었거나 경로가 바뀐 경우다. 현재 컴퓨터에서
`.venv_hand`를 다시 만들고 활성화한 뒤 패키지를 재빌드한다. 다른 컴퓨터의
venv 디렉터리를 복사해서 해결하지 않는다.

### `hand_landmarker.task`를 찾지 못함

모델 파일 위치를 확인하고 다시 빌드한다. 또는 절대경로를 직접 전달한다.

```bash
ros2 run hand_teleop hand_tracker_node_ver_depth \
  --ros-args \
  -p model_path:=/absolute/path/hand_landmarker.task
```

### 웹캠을 열 수 없음

```bash
v4l2-ctl --list-devices
ls -l /dev/video*
```

올바른 인덱스로 실행한다.

```bash
ros2 run hand_teleop hand_tracker_node_ver_depth \
  --ros-args \
  -p camera_index:=2
```

### RG2 연결 실패

- RG2/Compute Box 전원 확인
- PC와 RG2 네트워크 대역 확인
- `192.168.1.1:502` 확인
- RG2를 사용하지 않는 통합 시험은 `gripper.enabled:=false` 사용

### 노드는 실행되지만 로봇이 움직이지 않음

- `dry_run` 기본값은 `true`다.
- 실로봇 시험에서만 `dry_run:=false`를 사용한다.
- Doosan bringup과 `/dsr01` 서비스가 실행 중인지 확인한다.
- `hand_detected`, `depth_valid`, `hand_position` 토픽을 확인한다.

### 손을 처음 보여도 X 방향이 움직이지 않음

상대 깊이 중립 보정이 완료되지 않았을 수 있다.

```bash
ros2 topic echo \
  /hand_teleop/depth_valid
```

편 손을 중립 거리에서 움직이지 않고 유지한다. 카메라 위치나 사용자가
바뀌었으면 추적 노드를 재시작해서 다시 보정한다.

---

## 21. 통합 전 최종 체크리스트

### 소프트웨어

- [ ] Ubuntu 22.04 / ROS 2 Humble
- [ ] `.venv_hand`를 새 컴퓨터에서 생성
- [ ] `PYTHONNOUSERSITE=1` 설정
- [ ] `requirements.txt` 설치
- [ ] `hand_landmarker.task` 존재
- [ ] Doosan 패키지 빌드 및 source
- [ ] `hand_teleop` 빌드 및 source
- [ ] 스무딩 단위 테스트 10개 통과

### 손 추적

- [ ] 올바른 `camera_index`
- [ ] X/Y 방향 확인
- [ ] 45프레임 중립 깊이 보정 완료
- [ ] `hand_detected` 확인
- [ ] `depth_valid` 확인
- [ ] fist OPEN/CLOSE 안정화 확인

### 실제 로봇

- [ ] 먼저 `dry_run:=true`
- [ ] 그다음 `gripper.enabled:=false`
- [ ] 이동 범위를 ±50 mm 이하부터 시험
- [ ] `maximum_step_mm:=0.5`부터 시험
- [ ] 비상정지 준비
- [ ] 초기 자세 이동 경로 확인
- [ ] 손 유실 감속·정지 확인
- [ ] 자동 홈 전체 궤적 확인
- [ ] J5/J6 관절 속도 확인
- [ ] 마지막에 RG2 활성화

---

## 22. 권장 통합 순서 요약

```text
1. 패키지와 모델 파일 전달
2. 새 PC에서 venv 생성
3. requirements 설치
4. Doosan + hand_teleop 빌드
5. 손 추적 노드만 실행
6. ROS 토픽 인터페이스 확인
7. 통합 노드가 토픽을 구독하도록 연결
8. 로봇 없이 dry run
9. 실로봇 + RG2 비활성 + 작은 범위
10. 손 유실 및 홈 복귀 시험
11. 마지막에 RG2 활성화
```

통합 중 가장 중요한 원칙은 `hand_position` 하나만 사용하지 않고
`hand_detected`, `depth_valid`, 메시지 수신 시간을 함께 검사하는 것이다.
