# 호텔 안내 로봇 구현 과정 및 기능 흐름

## 1. 문서 목적

이 문서는 Doosan Robotics M0609, Intel RealSense D435i, OnRobot RG2,
일반 2D 웹캠을 이용해 구현한 기능들의 개발 과정과 현재 동작 구조를
정리한다.

다음 내용을 포함한다.

- 각 노드와 보조 모듈의 역할
- 입력과 출력
- 내부 처리 순서
- 개발 중 발생한 문제
- 문제의 원인과 해결 방식
- 사용한 기술과 알고리즘
- 현재 남아 있는 한계와 안전상 주의점

분석 대상은 다음 파일이다.

```text
beverage/bev/hotel_vision/opencv_center_test.py
beverage/bev/hotel_vision/move_to_bev_opencv.py

hand_teleop/hand_teleop/hand_tracker_node.py
hand_teleop/hand_teleop/hand_tracker_node_ver_depth.py
hand_teleop/hand_teleop/hand_follow_robot_node_ver_depth.py
hand_teleop/hand_teleop/onrobot.py

cobot_pjt/peter/move_plate.py
```

`onrobot.py`는 ROS 2 노드가 아니라 RG2 통신을 담당하는 보조 드라이버다.

---

## 2. 전체 시스템 구조

### 2.1 음료 픽앤플레이스

```text
YOLO 음료 검출
    │
    │ BeveragePosition
    │ class / confidence / BASE X,Y,Z
    ▼
move_to_bev_opencv
    │
    ├─ 거친 캔 좌표로 안전 접근
    ├─ TCP 수직 정렬
    ├─ RealSense RGB/Depth 수집
    ├─ OpenCV 원 중심 정밀 검출
    ├─ Camera 좌표 → BASE 좌표 변환
    ├─ 캔 수직 파지
    └─ 안전 높이 운반 및 트레이 배치
```

### 2.2 손동작 로봇 추종

```text
일반 USB 2D 웹캠
    │
    ▼
hand_tracker_node_ver_depth
    │
    ├─ 손 화면 X/Y
    ├─ 손바닥 크기 기반 상대 깊이 Z
    ├─ 손 검출 여부
    ├─ 주먹 여부
    └─ UI용 주석 영상
    │
    │ ROS 2 topics
    ▼
hand_follow_robot_node_ver_depth
    │
    ├─ 손 좌표 → BASE XYZ 오프셋
    ├─ 작업범위 제한
    ├─ 속도·가속도 스무딩
    ├─ 손 유실 복구 상태 머신
    ├─ Doosan servol 제어
    └─ 주먹 상태 → RG2 Modbus 제어
```

### 2.3 접시 이동 시험

```text
move_plate 실행
    │
    ├─ 고정 home pose
    ├─ 고정 접시 파지 관절 pose
    ├─ RG2 close
    ├─ BASE +X 상대 이동
    └─ RG2 open
```

`move_plate`는 센서나 토픽을 사용하지 않고 정해진 명령을 한 번 실행하는
초기 시험 코드다.

---

## 3. `opencv_center_test.py`

### 3.1 만든 목적

RealSense가 그리퍼 정중앙에 장착된 것이 아니어서, 카메라 영상의 중앙과
실제 그리퍼 파지 중심이 일치하지 않았다.

캔을 그리퍼의 실제 파지 중심 아래에 놓았을 때 캔은 영상 중앙보다 오른쪽
아래에 나타났다. 따라서 다음 값을 실험적으로 측정할 필요가 있었다.

- 실제 파지 중심에 대응하는 영상 픽셀
- 원 검출을 실행할 ROI의 위치와 크기
- Hough Circle 민감도
- 검출 원의 Depth 안정성
- 검출된 원의 실제 지름이 캔 크기와 비슷한지

이 노드는 로봇을 전혀 움직이지 않는다. RealSense 화면과 검출 결과만
확인하는 보정·진단 노드다.

### 3.2 입력

```text
/camera/camera/color/image_raw
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/color/camera_info
```

- `color/image_raw`: BGR 컬러 영상
- `aligned_depth_to_color/image_raw`: 컬러 영상에 정렬된 Depth
- `camera_info`: `fx`, `fy`, `ppx`, `ppy` 카메라 내부 파라미터

### 3.3 ROI 설정

현재 설정은 다음과 같다.

```python
ROI_CENTER_X_RATIO = 0.6508
ROI_CENTER_Y_RATIO = 0.8236
ROI_WIDTH_RATIO = 0.32
ROI_HEIGHT_RATIO = 0.40
ROI_BOTTOM_LIMIT_RATIO = 0.99
```

픽셀이 아니라 영상 크기에 대한 비율을 사용하므로 해상도가 바뀌어도 같은
상대 위치에 ROI를 만들 수 있다.

1280×720 영상에서는 ROI 기준점이 대략 `(833, 593)`이 된다.

### 3.4 원 검출 흐름

```text
컬러 영상 수신
    │
    ├─ ROI 자르기
    ├─ BGR → Gray
    ├─ CLAHE 명암 대비 보강
    ├─ Median Blur 노이즈 제거
    ├─ Hough Circle 후보 검출
    ├─ 원 전체가 ROI 안에 있는지 검사
    ├─ 원 내부 Depth 추출
    ├─ 실제 지름 추정
    └─ 최종 캔 후보 선택
```

최종 원은 다음 점수가 가장 작은 후보로 선택한다.

```text
ROI 기준점까지 거리
    +
예상 캔 지름 66 mm와의 차이에 대한 penalty
```

실제 추정 지름이 45~85mm 범위를 벗어난 후보는 선택 대상에서 제외한다.

### 3.5 Depth 안정화

캔 중앙 한 픽셀의 Depth만 읽으면 금속 반사 때문에 0이나 큰 이상값이 나올
수 있다.

현재 구현은 다음 방식으로 Depth를 안정화한다.

1. 검출 원 반지름의 50% 영역을 사용한다.
2. 원형 mask 내부 픽셀만 선택한다.
3. 100~2000mm 범위의 Depth만 남긴다.
4. 중앙값을 계산한다.
5. MAD(Median Absolute Deviation)로 이상치를 제거한다.
6. 남은 값의 중앙값을 최종 Depth로 사용한다.

### 3.6 화면 표시 의미

- 자홍색 사각형: ROI
- 흰색 십자: 영상 전체 중앙
- 빨간 십자: 실제 파지 중심으로 보정한 ROI 기준점
- 노란 원: 검출된 모든 원 후보
- 초록 원: 최종 선택된 캔
- 빨간 점: 선택된 캔 중심

### 3.7 중심 기록

`S` 키를 누르면 원 중심과 Depth를 20프레임 기록한다.

다음 조건을 만족하지 못하면 기록을 폐기한다.

- 중심의 최대 흔들림이 8픽셀 이하
- Depth 최대 범위가 20mm 이하

정상 기록이면 다음 값을 출력한다.

```text
pixel=(x, y)
ratio=(x / width, y / height)
depth=...
```

마우스로 화면을 클릭해도 클릭한 픽셀과 비율을 로그로 확인할 수 있다.

### 3.8 개발 중 문제와 해결

| 문제 | 원인 | 해결 |
|---|---|---|
| 영상 중앙과 파지 중심 불일치 | 손목 카메라 장착 오프셋 | 실제 파지 위치에서 ROI 중심을 직접 측정 |
| Depth가 튀거나 0이 나옴 | 금속 캔 반사 | 원 내부 다중 픽셀 중앙값과 MAD 필터 |
| 식혜 캔 원 검출 누락 | 원 윤곽이 약함 | `HOUGH_PARAM2`를 낮춰 민감도 증가 |
| 여러 원이 동시에 검출됨 | 캔과 주변 원형 물체 | ROI 중심 거리와 실제 지름을 함께 평가 |
| RGB와 Depth가 엇갈림 | 비동기 ROS 토픽 | aligned Depth, 해상도 검사, timestamp 차이 제한 |

### 3.9 한계

- 촬영 높이가 달라지면 카메라 시차 때문에 ROI 중심이 달라질 수 있다.
- 여러 캔이 ROI 안에 들어오면 의도한 종류가 아니라 ROI 기준점에 가까운 캔을
  선택할 수 있다.
- Hough 파라미터를 너무 민감하게 설정하면 바닥이나 캔 홀더를 원으로
  오검출할 수 있다.

---

## 4. `move_to_bev_opencv.py`

### 4.1 만든 목적

처음에는 YOLO가 발행한 BASE X/Y/Z만 이용해 캔으로 이동했다. 하지만 YOLO
bounding box의 대표점은 그리퍼 파지 중심과 정확히 일치하지 않았다.

발생한 문제는 다음과 같다.

- 캔 옆으로 이동함
- 여러 캔 사이에서 수 cm 오차가 발생함
- 측면 파지가 위치 오차에 민감함
- 손목 카메라와 TCP 사이의 오프셋이 반영되지 않음

따라서 YOLO 좌표는 캔 근처까지 가는 거친 좌표로만 사용하고, 캔 위에서
OpenCV와 Depth로 다시 측정하는 2단계 구조로 변경했다.

### 4.2 입력

#### 음료 거친 좌표

```text
topic: /checkin/beverage_position
type: checkin_interfaces/msg/BeveragePosition
```

사용 필드:

```text
class_name
confidence
base_x
base_y
base_z
```

#### RealSense

```text
/camera/camera/color/image_raw
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/color/camera_info
```

### 4.3 실행 특성

이 노드는 서비스 서버가 아니다.

실행 후 새 음료 좌표 하나를 받으면 파지와 배치를 한 번 수행하고 종료한다.
좌표를 계속 받아 반복 서비스하는 구조는 아직 구현돼 있지 않다.

### 4.4 전체 실행 순서

#### 1단계: DSR 초기화

1. `/dsr01` 네임스페이스에 로봇용 ROS 노드를 생성한다.
2. `DR_init.__dsr__node`에 생성한 노드를 등록한다.
3. 그 이후에 `DSR_ROBOT2`를 import한다.

이 순서가 필요한 이유는 `DSR_ROBOT2`가 import되는 순간 ROS 서비스
클라이언트를 만들기 때문이다.

등록 전 import하면 다음 오류가 발생한다.

```text
AttributeError: 'NoneType' object has no attribute 'create_client'
```

#### 2단계: 위험 설정 사전 검사

로봇을 움직이기 전에 다음을 확인한다.

- 트레이 접근 pose가 작업범위 안인지
- release Z가 접근 Z보다 낮은지
- 안전 운반 높이가 작업범위 안인지
- `T_flange_camera.npy`가 정상적인 4×4 행렬인지
- 회전행렬이 직교행렬이고 determinant가 1인지

#### 3단계: 초기 자세와 그리퍼

```text
INITIAL_JOINT_POSE로 movej
    │
    ▼
RG2 open
    │
    ▼
음료 좌표 subscriber 생성
```

subscriber를 초기 이동 후 만드는 이유는 detector가 이전에 발행해 둔 오래된
좌표로 로봇이 갑자기 움직이지 않도록 하기 위해서다.

`VOLATILE` QoS로 새 메시지만 받는다.

#### 4단계: 거친 좌표 검사

다음 조건을 만족하는 첫 좌표만 사용한다.

- 모든 값이 유한수
- confidence 0.50 이상
- 설정된 BASE X/Y/Z 범위 안
- BASE XY 반경 850mm 이하

#### 5단계: 캔 위 안전 접근

낮은 위치에서 캔까지 대각선으로 이동하지 않도록 경로를 분리했다.

```text
현재 XY에서 안전 높이까지 BASE +Z 상승
    │
    ▼
안전 높이에서 TCP 수직 정렬
    │ ABC=[90, -180, -90]
    ▼
안전 높이를 유지하며 캔 X/Y 위로 이동
    │
    ▼
캔 Z + ROUGH_APPROACH_Z_OFFSET_MM 촬영 높이로 하강
```

현재 주요 값:

```python
ROUGH_APPROACH_Z_OFFSET_MM = 80.0
SAFE_CLEARANCE_ABOVE_CAN_MM = 100.0
```

#### 6단계: OpenCV 정밀 검출

로봇 이동 후 카메라 흔들림이 가라앉도록 0.8초 기다린다.

그 후 새 프레임에서 다음 값을 5개 모은다.

- 원 중심 X/Y
- 원 반지름
- 원 내부 Depth

다음 안정성 조건을 만족해야 결과를 사용한다.

- 중심 흔들림 8픽셀 이하
- Depth 범위 15mm 이하
- 반지름 최대 상대 편차 20% 이하

#### 7단계: 픽셀을 카메라 XYZ로 변환

aligned Depth와 카메라 내부 파라미터를 이용한다.

```text
Xc = (u - ppx) × depth / fx
Yc = (v - ppy) × depth / fy
Zc = depth
```

결과는 RealSense optical frame 기준 좌표다.

#### 8단계: 카메라 XYZ를 BASE XYZ로 변환

```text
P_base =
    T_base_flange
  × T_flange_camera
  × P_camera
```

- `T_base_flange`: 로봇의 현재 Tool Flange pose
- `T_flange_camera`: hand-eye calibration 결과
- `P_camera`: RealSense optical frame의 캔 좌표

Doosan `posx`의 ABC는 ZYZ Euler 각으로 회전행렬로 변환한다.

이 방식은 과거 `trans()`에 BASE 절대 pose와 TOOL 상대 변위를 잘못 섞어
비정상적으로 큰 좌표가 계산되던 문제를 대신한다.

#### 9단계: 정밀 XY 보정

현재 TCP Z와 수직 ABC는 유지하고 BASE X/Y만 OpenCV로 계산한 캔 중심에
맞춘다.

오검출 방지를 위해 다음을 검사한다.

- 기존 위치와 정밀 위치의 XY 보정량 80mm 이하
- 현재 TCP와 캔 윗면의 수직 간격 20~300mm

#### 10단계: 최종 하강

최종 파지 TCP Z는 다음과 같다.

```text
grasp_target_z =
    검출한 캔 윗면 BASE Z
  + GRASP_TCP_Z_OFFSET_FROM_CAN_TOP_MM
```

현재 값:

```python
GRASP_TCP_Z_OFFSET_FROM_CAN_TOP_MM = -30.0
```

계산된 하강량이 5~120mm일 때만 하강한다.

과거 다음 오류는 이 보호 조건에서 발생했다.

```text
계산한 하강량 -16.4 mm가 허용 범위 밖입니다.
캔 윗면 높이 차 -7.4 mm가 허용 범위 밖입니다.
```

TCP 원점, 촬영 높이, 캔 윗면 Z, 그리퍼 파지 중심 오프셋의 관계가 맞지 않아
목표가 현재 TCP보다 위로 계산됐기 때문이다.

#### 11단계: RG2 파지와 상승

현재 설정:

```text
open width: 70.0 mm
grip width: 50.0 mm
force: 20.0 N
```

기존 RG2 API는 다음 raw 단위를 사용한다.

```text
width raw = mm × 10
force raw = N × 10
```

따라서 `force=200`은 20N이다.

파지 명령 이후 다음 상태를 확인한다.

- busy
- grip detected
- safety switch
- safety circuit
- safety error

파지 실패 시 낮은 위치에 계속 머물지 않도록 하강한 거리만큼 다시 상승한다.

#### 12단계: 트레이 운반과 배치

캔을 든 상태에서 바로 트레이로 대각선 이동하지 않는다.

```text
현재 XY에서 안전 운반 높이까지 상승
    │
    ▼
수직 파지 ABC를 유지한 채 트레이 X/Y 위로 수평 이동
    │
    ▼
트레이 위 안전 높이에서 배치용 ABC로 회전
    │
    ▼
PLACE_APPROACH_POSE로 하강
    │
    ▼
PLACE_RELEASE_Z_MM까지 천천히 하강
    │
    ▼
RG2 open
    │
    ▼
안전 운반 높이로 후퇴
```

안전 운반 높이는 다음 중 가장 높은 값이다.

```text
현재 파지 후 TCP Z
PLACE_APPROACH_POSE Z
PLACE_RELEASE_Z_MM + PLACE_RETREAT_CLEARANCE_MM
```

### 4.5 개발 중 문제와 해결

| 문제 | 원인 | 해결 |
|---|---|---|
| `g_node=None` | DSR import 순서 오류 | 노드 등록 후 `DSR_ROBOT2` import |
| NumPy truth value ambiguous | 배열을 bool 조건으로 사용 | pose를 6개 실수 배열로 정규화 |
| `trans()`가 거대한 좌표 반환 | 좌표계와 절대/상대 pose 혼합 | 명시적인 4×4 동차변환 |
| YOLO 위치로 가면 캔 옆으로 이동 | bounding box 대표점 오차 | YOLO 거친 접근 + OpenCV 정밀 보정 |
| 하강량이 음수 | TCP와 캔 윗면 offset 불일치 | 명시적인 목표 Z 식과 하강량 범위 검사 |
| RG2 busy 문구 반복 | 그리퍼가 실제 이동 중 | 명령 1회 후 상태 polling |
| 캔 파지 여부 불명확 | 목표 폭만 확인 | RG2 grip detected 비트 확인 |
| 파지 후 대각선 이동 | 낮은 장애물 충돌 가능성 | 상승·수평이동·하강으로 경로 분리 |
| 오류 시 캔 자동 낙하 위험 | 예외 처리 중 자동 open | `holding_object` 상태에서는 자동 open 금지 |

### 4.6 현재 주의점

현재 다음 기능이 실제로 활성화돼 있다.

```python
ENABLE_FINAL_DESCENT_AND_GRIP = True
ENABLE_TRAY_PLACE = True
```

또한 현재 `PLACE_APPROACH_POSE`는 이전에 제공했던 커피 좌표와 동일하다.
실제 트레이 접근 좌표가 맞는지 다시 확인해야 한다.

현재 그리퍼 설정도 처음 논의했던 62~64mm, 약 3N이 아니라 50mm, 20N이다.

`GRASP_TCP_Z_OFFSET_FROM_CAN_TOP_MM=-30`은 범용값이 아니다. 현재 TCP가
실제 손가락 파지 중심에 정의돼 있는지 실측해야 한다.

이 코드는 `movel` 구간을 안전하게 나눈 것이지 MoveIt 충돌 회피를 수행하는
것은 아니다.

---

## 5. `hand_tracker_node.py`

### 5.1 만든 목적

사람 손 인식과 로봇 제어를 하나의 노드에 넣지 않고 분리했다.

이 노드는 영상 인식만 담당하며 로봇을 직접 움직이지 않는다.

분리한 이유:

- 카메라나 MediaPipe 오류가 로봇 API 호출에 바로 영향을 주지 않음
- 인식 노드만 별도로 시험 가능
- UI와 로봇 실행 계층을 독립적으로 통합 가능
- 실제 로봇 없이 토픽만 검사 가능

### 5.2 입력

- 일반 USB 2D 웹캠
- MediaPipe `hand_landmarker.task` 모델

### 5.3 출력 토픽

```text
/hand_teleop/hand_position
    geometry_msgs/PointStamped
    x: 화면 왼쪽 0.0 → 오른쪽 1.0
    y: 화면 위쪽 0.0 → 아래쪽 1.0
    z: 0.0

/hand_teleop/hand_detected
    std_msgs/Bool

/hand_teleop/fist
    std_msgs/Bool
```

### 5.4 프레임 처리 흐름

```text
OpenCV VideoCapture
    │
    ├─ V4L2 backend 시도
    ├─ 실패 시 기본 OpenCV backend 재시도
    ├─ 필요 시 좌우 반전
    ├─ BGR → RGB
    ├─ contiguous NumPy 배열 변환
    └─ MediaPipe VIDEO 모드 입력
```

MediaPipe VIDEO 모드는 프레임 timestamp가 계속 증가해야 한다. 같은 millisecond
값이 나오면 이전 timestamp보다 1 증가시킨다.

### 5.5 손 중심 계산

손가락 끝 한 점은 손을 펴거나 주먹을 쥘 때 크게 움직인다. 그래서 다음 다섯
랜드마크 평균을 손바닥 중심으로 사용한다.

```text
0: 손목
5: 검지 MCP
9: 중지 MCP
13: 약지 MCP
17: 소지 MCP
```

좌표는 0~1 범위로 제한한다.

### 5.6 X/Y 필터

손 중심의 작은 떨림을 줄이기 위해 EMA를 사용한다.

```text
filtered =
    alpha × raw
  + (1 - alpha) × previous
```

기본값:

```text
filter_alpha = 0.25
```

값이 작을수록 부드럽지만 반응이 느려진다.

### 5.7 주먹 판정

엄지를 제외한 네 손가락을 사용한다.

각 손가락에 대해 다음 거리를 비교한다.

```text
손바닥 중심 → PIP
손바닥 중심 → 손끝
```

손끝 거리가 PIP 거리의 설정 비율 이하이면 접힌 손가락으로 판단한다.

기본적으로 네 손가락 중 세 개 이상이 접히면 raw fist다.

### 5.8 주먹 상태 안정화

한 프레임의 오검출로 그리퍼가 움직이지 않도록 연속 프레임을 확인한다.

```text
주먹 8프레임 연속 → stable fist = True
편 손 3프레임 연속 → stable fist = False
```

30fps 기준으로 약 0.27초와 0.10초다.

### 5.9 손 유실 처리

손이 보이지 않으면:

- `(0, 0)` 같은 가짜 위치를 발행하지 않는다.
- `hand_detected=False`를 발행한다.
- `fist=False`를 발행한다.
- X/Y 필터를 초기화한다.
- 주먹 안정화 카운터를 초기화한다.

재검출 순간 이전 손의 필터값 때문에 좌표가 튀는 것을 방지한다.

### 5.10 개발 중 문제와 해결

| 문제 | 해결 |
|---|---|
| 손 중심이 손가락 모양에 따라 움직임 | 손목과 네 MCP 평균 사용 |
| X/Y가 떨림 | EMA 저역통과 필터 |
| 주먹이 한 프레임마다 바뀜 | 8/3 프레임 hysteresis |
| 모델 파일 누락 | parameter → install share → source models 순서로 탐색 |
| 카메라 번호 오류 | V4L2 fallback과 실제 capture 정보 로그 |
| 로봇 없이 시험하기 어려움 | 영상 인식과 로봇 제어 노드 분리 |

---

## 6. `hand_tracker_node_ver_depth.py`

### 6.1 만든 목적

기존 2D 손 추적에 사람 손의 앞뒤 움직임을 추가하기 위해 만들었다.

로봇팔의 RealSense는 벽과 종이를 바라보고, 사람은 별도의 일반 2D 웹캠 앞에서
손을 움직이는 구성이다.

일반 웹캠에는 거리 센서가 없으므로 손바닥의 화면상 크기를 상대 깊이로
사용한다.

이 Z는 mm 단위 실제 거리가 아니다.

### 6.2 상속 구조

`DepthHandTrackerNode`는 `HandTrackerNode`를 상속한다.

부모 노드에서 다음 기능을 그대로 재사용한다.

- 카메라 초기화
- MediaPipe Hand Landmarker
- X/Y 손 중심 계산
- X/Y EMA
- 주먹 판정
- detected/fist/position 토픽
- 디버그 그림

깊이 버전은 다음 기능만 추가한다.

- 손바닥 크기 계산
- 중립 거리 보정
- 상대 깊이 Z
- `depth_valid`
- UI용 `annotated_image`

### 6.3 손바닥 크기 계산

```text
손바닥 폭: landmark 5 ↔ 17
손바닥 높이: landmark 0 ↔ 9
```

대표 크기는 기하평균이다.

```text
palm_size = sqrt(width × height)
```

한 방향의 기울기 변화가 깊이에 미치는 영향을 줄이기 위해 폭 또는 높이 하나만
사용하지 않았다.

### 6.4 중립 깊이 보정

노드 시작 후 편 손을 평소 조종 거리에서 기본 45프레임 유지한다.

보정 흐름:

1. 유효한 손바닥 크기만 모은다.
2. 주먹이 아닌 프레임만 사용한다.
3. 45개 샘플의 중앙값을 기준값으로 사용한다.
4. 10~90 percentile 범위를 계산한다.
5. 변화량이 기준값의 15%를 넘으면 사용자가 움직였다고 판단한다.
6. 불안정한 샘플을 폐기하고 다시 보정한다.

### 6.5 상대 깊이 계산

```text
size_delta =
    current_palm_size / reference_palm_size - 1
```

기본 설정:

```text
deadzone = ±4%
full scale = ±35%
output = -1.0 ~ +1.0
depth EMA alpha = 0.20
```

- 손이 커지면 카메라에 가까워짐 → Z 양수
- 손이 작아지면 카메라에서 멀어짐 → Z 음수
- ±4% 이내 변화는 Z=0

### 6.6 추가 토픽

```text
/hand_teleop/depth_valid
    std_msgs/Bool

/hand_teleop/annotated_image
    sensor_msgs/Image
    encoding: bgr8
```

깊이 보정 전에도 X/Y는 발행한다. 이때 Z는 0이고 `depth_valid=False`다.

따라서 follower는 Z가 0인지 보는 것이 아니라 반드시 `depth_valid`를 함께
확인해야 한다.

### 6.7 UI 영상

OpenCV 디버그 창에 그린 다음 정보를 ROS Image로도 발행한다.

- 손 랜드마크
- 손바닥 중심
- raw/stable fist
- raw/filtered depth
- NEAR/FAR/NEUTRAL
- 손바닥 크기와 기준값
- 깊이 막대

`cv_bridge`를 사용하지 않고 `sensor_msgs/Image` 필드를 직접 채운다.

이 방식은 당시 NumPy 2.x와 ROS Humble의 NumPy 1.x 기반 `cv_bridge` 사이
ABI 충돌을 피하기 위해 적용했다.

이미지 publisher queue depth는 1이다. UI가 늦어져도 오래된 프레임이 계속
쌓이지 않게 한다.

### 6.8 한계

- 실제 거리센서가 아니므로 mm 단위 제어에 사용할 수 없다.
- 사람 손 크기, 손의 기울기, 카메라 해상도에 영향을 받는다.
- 사용자가 바뀌거나 카메라 위치가 바뀌면 다시 보정해야 한다.
- 손바닥을 심하게 기울이면 앞뒤로 움직이지 않아도 깊이가 바뀔 수 있다.
- 벽이나 종이까지의 실제 충돌 거리 확인에는 사용할 수 없다.

---

## 7. `hand_follow_robot_node_ver_depth.py`

### 7.1 만든 목적

`hand_tracker_node_ver_depth`의 정규화된 손 좌표를 실제 Doosan M0609 TCP
목표로 바꾸는 실행 계층이다.

카메라 영상은 처리하지 않는다. 구독 콜백에서는 최신 값과 수신 시간만
저장하고, 실제 로봇 명령은 고정 주기의 제어 타이머에서 수행한다.

### 7.2 입력 토픽

```text
/hand_teleop/hand_position
/hand_teleop/hand_detected
/hand_teleop/depth_valid
/hand_teleop/fist
```

### 7.3 초기화 흐름

실제 실행 모드:

```text
RG2 Modbus 연결 확인
    │
    ▼
initial_joint_pose로 movej
    │
    ▼
현재 TCP pose(BASE) 읽기
    │
    ▼
추종 원점으로 저장
    │
    ▼
10 Hz 제어 타이머 시작
```

초기 관절 자세:

```text
[-90, 0, 90, -90, 90, 90]
```

`dry_run=True`가 기본이므로 실제 명령 없이 좌표와 제스처 계산을 먼저 시험할
수 있다.

### 7.4 `DR_init.__dsr__node` 오류 해결

클래스 메서드 안에서 다음 코드를 직접 사용하면:

```python
DR_init.__dsr__node = self
```

Python name mangling 때문에 의도하지 않은 속성이 생성될 수 있다.

현재는 다음 방식으로 정확한 이름을 설정한다.

```python
setattr(DR_init, "__dsr__node", self)
```

그 이후에 `DSR_ROBOT2`를 import한다.

### 7.5 손 좌표를 로봇 좌표로 변환

현재 매핑:

```text
화면 오른쪽 +X  → BASE +Y
화면 아래쪽 +Y  → BASE -Z
손이 카메라 쪽 +Z → BASE -X
```

현재 워크스페이스의 기본 범위:

```text
X 전진 거리: 0~250 mm
Y offset: -250~250 mm
Z offset: -250~250 mm
```

손이 중립보다 카메라에서 멀어져 Z가 음수가 되면 X 전진량은 0으로 제한한다.
초기 위치 반대 방향으로 넘어가지는 않는다.

### 7.6 누적 오차 방지

목표를 직전 로봇 위치에 계속 더하지 않는다.

매 주기 다음과 같이 초기 TCP에서 계산한다.

```text
target_x = origin_x + x_offset
target_y = origin_y + y_offset
target_z = origin_z + z_offset
target_abc = origin_abc
```

따라서 상대 이동 명령의 누적 오차가 발생하지 않는다.

TCP의 A/B/C는 초기 자세로 유지한다.

### 7.7 로봇이 탁탁 움직였던 문제

초기 follower는 손 목표가 바뀔 때마다 새 목표 쪽으로 큰 pose 변화를 보냈다.

발생한 문제:

- 손 인식 노이즈에 따라 목표 방향이 자주 바뀜
- X/Y/Z 축을 각각 제한해 대각선 전체 이동량이 커짐
- 속도가 한 주기 만에 반전됨
- Cartesian 작은 변화가 역기구학에서 J6의 큰 속도 변화로 나타날 수 있음
- 로봇에서 탁탁 걸리는 느낌과 관절 속도 제한 오류가 발생함

### 7.8 XYZ 벡터 이동 제한

기존 축별 제한을 XYZ 벡터 전체 거리 제한으로 변경했다.

```text
delta = desired_xyz - previous_xyz
distance = ||delta||

distance > maximum_step_mm이면
delta 전체에 maximum_step_mm / distance를 곱함
```

이동 방향은 유지하면서 대각선에서도 전체 이동량이 제한값을 넘지 않는다.

`maximum_step_mm` 기본값도 5mm에서 1mm로 줄였다.

### 7.9 속도·가속도 스무딩

제어 흐름:

1. 현재 명령 pose에서 최종 목표까지의 거리와 방향을 계산한다.
2. 목표 속도를 계산한다.
3. 목표 근처에서는 정지 가능 속도 `sqrt(2as)`로 감속한다.
4. 이전 속도에서 새 속도로 바뀔 수 있는 벡터 크기를 제한한다.
5. 새 pose를 계산한다.
6. step과 작업공간을 다시 제한한다.

목표 속도는 다음 값 중 가장 작은 값을 사용한다.

```text
외부 명령 속도 상한
정지 가능 속도 sqrt(2 × acceleration × remaining_distance)
남은 거리를 한 주기에 넘지 않는 속도
```

가속도 제한:

```text
||new_velocity - previous_velocity||
    ≤ command_acceleration × control_period
```

새 pose:

```text
new_position =
    previous_position + smoothed_velocity × control_period
```

기본 설정:

```text
control rate = 10 Hz
maximum step = 1 mm
실질 목표 속도 상한 ≈ 10 mm/s
command acceleration = 20 mm/s²
한 주기 최대 속도 변화 = 2 mm/s
```

`servo_linear_velocity=30mm/s`여도 `1mm × 10Hz = 10mm/s` 제한이 먼저
적용된다.

이 구현은 가속도 제한 스무딩이다. 가속도 자체의 변화율까지 제한하는 jerk
스무딩은 아직 구현돼 있지 않다.

### 7.10 스무딩 시험

`test/test_motion_smoothing.py`에서 다음을 확인한다.

- 첫 명령이 정지 속도에서 서서히 가속
- 전체 벡터 속도 제한
- 전체 벡터 가속도 제한
- 대각선 이동거리 제한
- 방향 반전 시 `+속도 → 0 → -속도`
- 목표 근처 오버슈트 방지
- 실제 타이머 간격 반영
- 잘못된 제한값 거부
- 화면 +X가 BASE +Y로 매핑되는지

### 7.11 손 유실 복구 상태 머신

```text
WAITING_FOR_HAND
    │ 유효한 손
    ▼
TRACKING
    │ 손 미검출 / depth invalid / topic timeout
    ▼
DECELERATING
    │ speedl(0), 실제 TCP 속도 확인
    ▼
HOLDING
    ├─ 손이 빨리 복귀 → RESUMING_TRACKING
    └─ 계속 유실
           ▼
    RETURNING_HOME
           │
           ▼
    WAITING_FOR_HAND
```

세부 흐름:

1. 손을 잃으면 `servol` 목표 발행을 중단한다.
2. `speedl([0,0,0,0,0,0])`을 발행한다.
3. `get_current_velx`로 실제 TCP 속도를 확인한다.
4. 선속도와 회전속도가 기준 이하인 상태를 여러 번 확인한다.
5. 현재 위치에서 설정 시간만큼 유지한다.
6. 손이 유예 시간 안에 돌아오면 실제 TCP를 다시 읽는다.
7. 읽은 실제 TCP부터 부드럽게 추종을 재개한다.
8. 손이 계속 없으면 저속 비동기 `amovej`로 초기 관절 자세에 복귀한다.
9. `check_motion`, `get_current_posj`, `get_current_posx`로 홈 도착을 확인한다.

fault나 종료에서는 자동 홈 복귀 대신 `DR_SSTOP`을 요청한다.

### 7.12 RG2 제스처 제어

```text
stable fist=False → OPEN
stable fist=True  → CLOSE
```

구독 콜백에서 Modbus 통신을 직접 실행하지 않는다.

```text
fist callback
    │
    ▼
Condition에 최신 OPEN/CLOSE 목표 저장
    │
    ▼
RG2 전용 작업 스레드
    │
    ├─ 이전 동작 busy 해제 대기
    ├─ 안전 상태 확인
    └─ Modbus 명령 1회 전송
```

별도 스레드를 사용하는 이유는 RG2 통신과 busy polling이 10Hz `servol`
제어 타이머를 막지 않게 하기 위해서다.

같은 손 상태가 계속 유지되면 같은 명령을 반복하지 않는다.

손을 잃었을 때는 물체가 떨어지지 않도록 마지막 그리퍼 상태를 유지한다.

손이 재검출된 직후에는 0.35초 동안 fist 결과를 사용하지 않는다. 주먹인데도
재검출 직후의 임시 `fist=False` 때문에 그리퍼가 먼저 열리는 것을 방지한다.

### 7.13 현재 남은 문제

- 실제 기본 그리퍼 값은 `OPEN=30mm/40N`, `CLOSE=1mm/40N`이지만 파일 상단
  설명은 아직 `80mm/30N`, `2mm/30N`이다.
- 경고 로그는 ±300mm라고 하지만 현재 워크스페이스 기본값은 ±250mm다.
- `cobot_ws`와 `~/COB2`의 follower 이동 범위가 서로 다르다.
- 자동 홈 복귀는 MoveIt 충돌 회피가 아닌 단순 관절 `amovej`다.
- `speedl(0)`에서 홈 `amovej`로 전환할 때 스트리밍 모션 종료 상태를 더
  명확하게 확인할 필요가 있다.
- soft stop 실패 시 0.5초 간격으로 재시도하지만 최대 횟수 제한은 없다.
- 고정 ABC 자세가 특정 위치에서 특이점이나 J6 속도 확대를 만들 수 있다.
- 작업범위 제한은 직육면체 범위 제한일 뿐 장애물 회피가 아니다.
- 최근 발생한 5초 heartbeat timeout은 이 Python 노드가 직접 발생시킨
  오류라기보다 `dsr_bringup2`/DRFL 프로세스 또는 네트워크 문제다.

---

## 8. `hand_teleop/onrobot.py`

### 8.1 역할

ROS 노드가 아니라 OnRobot RG2용 최소 Modbus TCP 드라이버다.

기존 `peter.onrobot.RG`는 폭과 힘을 raw 정수로 받기 때문에 단위 혼동이
발생하기 쉬웠다.

새 드라이버는 사용자에게 mm와 N을 받고 내부에서 raw 값으로 변환한다.

### 8.2 연결

```text
IP: 192.168.1.1
port: 502
Modbus unit ID: 65
```

연결 실패나 Modbus 오류 응답은 Python 예외로 변환한다.

### 8.3 단위 변환

```text
width raw = width mm × 10
force raw = force N × 10
```

예:

```text
80.0 mm → raw 800
30.0 N  → raw 300
```

입력 범위:

```text
width: 0~110 mm
force: 3~40 N
```

### 8.4 이동 명령

Modbus holding register 0부터 다음 값을 기록한다.

```text
[force_raw, width_raw, 16]
```

- register 0: 목표 힘
- register 1: 목표 폭
- register 2: 제어 명령
- control 16: fingertip offset을 적용한 grip 명령

### 8.5 상태 레지스터

register 268의 비트를 다음 상태로 해석한다.

- busy
- grip detected
- safety switch 1 pushed
- safety circuit 1 triggered
- safety switch 2 pushed
- safety circuit 2 triggered
- safety error

`grip_detected`는 안전 버튼이 눌렸다는 뜻이 아니다.

그리퍼가 목표 폭으로 이동하는 과정에서 내부 또는 외부 파지를 감지했다는
상태다. 안전 스위치와 안전 회로는 별도의 비트다.

### 8.6 한계

- 이 드라이버 자체는 명령 완료까지 기다리지 않는다.
- busy polling과 timeout은 follower 작업 스레드가 담당한다.
- follower는 현재 `grip_detected`를 CLOSE 성공 판정에 사용하지 않는다.
- `pymodbus.client.sync`는 pymodbus 2.x API이므로 pymodbus 3.x에서는 import
  경로가 다를 수 있다.

---

## 9. `move_plate.py`

### 9.1 목적

접시를 고정된 관절 자세에서 잡고 고정 방향으로 이동하는 초기 기능 시험
코드다.

토픽이나 서비스 요청을 기다리지 않는다. 실행 즉시 정해진 순서를 한 번
수행하고 종료한다.

### 9.2 실행 순서

```text
ROS 2 초기화
    │
    ▼
/dsr01/move_plate 노드 생성
    │
    ▼
DR_init.__dsr__node 등록
    │
    ▼
DSR_ROBOT2 import
    │
    ▼
기존 peter.onrobot.RG 연결
    │
    ├─ open
    └─ 바로 close
    │
    ▼
home_position으로 movej
    │
    ▼
gripper open
    │
    ▼
grap_plate_position으로 movej
    │
    ▼
gripper close
    │
    ▼
1초 wait
    │
    ▼
BASE +X 508.63 mm 상대 movel
    │
    ▼
gripper open
    │
    ▼
연결 종료
```

### 9.3 `ros2 run`에 나타나지 않았던 문제

`cobot_pjt`는 `ament_cmake` 기반 패키지다.

`setup.py`에 entry point만 추가해도 현재 빌드 구조에서는 실행 파일이
설치되지 않을 수 있다.

현재는 다음 두 가지가 추가돼 있다.

```text
scripts/move_plate
CMakeLists.txt의 install(PROGRAMS ...)
```

빌드 후 workspace를 source하면 다음처럼 실행할 수 있다.

```bash
ros2 run cobot_pjt move_plate
```

### 9.4 `movel` 좌표 문제

현재 명령:

```python
movel(
    posx(508.63, 0, 0, 0, 0, 0),
    VELOCITY,
    ACC,
    ref=DR_BASE,
    mod=DR_MV_MOD_REL,
)
```

`DR_MV_MOD_REL`이므로 절대 BASE X=508.63mm로 가는 명령이 아니다.

현재 TCP에서 BASE +X 방향으로 508.63mm 상대이동하는 명령이다.

나머지 Y/Z/A/B/C가 0이므로 상대 Y/Z 이동과 상대 자세 변화는 없다.

508.63mm는 한 번의 상대이동으로는 큰 거리이므로 도달 가능성, 특이점,
충돌을 확인해야 한다.

### 9.5 현재 남은 문제

- 시작 직후 `open_gripper()`와 `close_gripper()`를 대기 없이 연달아 호출한다.
- RG2가 busy이면 두 번째 명령이 무시될 수 있다.
- 개선된 `hand_teleop.onrobot.RG2`가 아니라 예전 `peter.onrobot.RG`를 쓴다.
- 기존 드라이버는 연결 성공과 Modbus 오류 응답을 충분히 검사하지 않는다.
- busy, grip detected, safety 상태를 확인하지 않는다.
- close 후 실제 완료가 아니라 고정 1초만 기다린다.
- `movej`와 `movel` 반환값을 검사하지 않는다.
- 예외 발생 시 그리퍼 연결 종료가 보장되지 않는다.
- dry run과 작업범위 제한이 없다.
- 접시를 든 뒤 안전 높이로 먼저 상승하는 단계가 없다.
- MoveIt 충돌 회피가 없다.

따라서 `move_plate`는 통합 운용 노드가 아니라 기능 확인용 초기 초안으로
분류하는 것이 적절하다.

---

## 10. 공통 개발 환경과 패키지 문제

### 10.1 ROS 2와 Python 가상환경

사용 환경:

```text
Ubuntu 22.04
ROS 2 Humble
Python 3.10
```

MediaPipe와 OpenCV 패키지 충돌을 줄이기 위해 Python venv를 사용했다.

ROS 2 Python 패키지를 계속 사용하기 위해 `--system-site-packages` 기반
venv를 사용하고, 사용자 전역 패키지의 오염을 줄이기 위해 다음 값을
사용했다.

```bash
export PYTHONNOUSERSITE=1
```

### 10.2 NumPy, OpenCV, MediaPipe 충돌

시험한 조합:

```text
Python 3.10
numpy==1.24.4
opencv-contrib-python==4.10.0.84
mediapipe==0.10.35
```

이 조합에서 손 검출과 annotated image 토픽이 정상 동작하는 것을 확인했다.

현재 `hand_teleop/requirements.txt`는 다른 조합을 기록하고 있으므로 팀
통합 전에 하나의 검증된 버전 조합으로 통일해야 한다.

### 10.3 Pylance에서 `dsr_msgs2` import를 찾지 못함

주요 원인:

- `/opt/ros/humble/setup.bash`를 source하지 않음
- workspace의 `install/setup.bash`를 source하지 않음
- VS Code가 ROS/venv와 다른 Python interpreter를 사용

이 오류는 코드 자체보다 개발 환경과 Python interpreter 경로 문제였다.

### 10.4 YOLO GPU 오류

사용 노트북의 RTX 5070 Ti Laptop GPU는 `sm_120` capability를 사용했지만,
설치된 PyTorch가 해당 CUDA architecture를 지원하지 않았다.

발생 오류:

```text
CUDA error: no kernel image is available for execution on the device
```

초기 해결은 YOLO inference를 CPU로 실행하는 방식이었다.

장기적으로 GPU를 사용하려면 해당 GPU architecture를 지원하는 PyTorch와
CUDA 조합을 설치해야 한다.

### 10.5 실행 파일이 `ros2 run`에 나타나지 않음

`ament_python` 패키지:

- `setup.py`의 `console_scripts`에 entry point 추가

`ament_cmake` 패키지:

- 실행 wrapper 추가
- `CMakeLists.txt`의 `install(PROGRAMS ...)`에 등록

공통적으로 변경 후 다음 과정이 필요하다.

```text
colcon build
    │
    ▼
source install/setup.bash
    │
    ▼
ros2 run <package> <executable>
```

---

## 11. 기능 발전 타임라인

### 11.1 음료 기능

```text
고정 좌표 movej/movel 시험
    │
    ▼
YOLO 음료 좌표 토픽 수신
    │
    ▼
BASE 좌표 접근
    │
    ▼
TOOL/BASE 좌표계 혼동 수정
    │
    ▼
측면 파지에서 수직 파지로 변경
    │
    ▼
opencv_center_test로 ROI 측정
    │
    ▼
Hough Circle + Depth 정밀 중심
    │
    ▼
Hand-eye Camera→BASE 변환
    │
    ▼
TCP 파지 offset 및 하강 안전검사
    │
    ▼
RG2 grip detected 확인
    │
    ▼
안전 운반 높이와 트레이 배치 추가
```

### 11.2 손 추종 기능

Git 이력상 주요 발전 과정:

```text
c0f5695
hand_teleop 패키지와 2D hand_tracker_node 생성
    │
    ▼
b47a328
손바닥 크기 기반 상대 깊이 tracker 추가
    │
    ▼
095cf83
depth follower 초기 구현
로봇이 툭툭 끊기는 문제 확인
    │
    ▼
b5e0117
주먹 기반 RG2 제어와 전용 Modbus 드라이버 추가
    │
    ▼
bf6c728
축별 제한 → XYZ 벡터 제한
maximum step 5 mm → 1 mm
    │
    ▼
260ed57
XYZ 속도 방향과 가속도 스무딩 추가
pytest 시험 추가
    │
    ▼
aaf5e9e
RG2 폭과 힘 기본값 조정
    │
    ▼
350c4f2
annotated image 토픽 추가
화면 +X → BASE +Y 방향 수정
```

---

## 12. 사용 기술 요약

### ROS 및 로봇

- Ubuntu 22.04
- ROS 2 Humble
- `rclpy`
- `ament_python`
- `ament_cmake`
- Doosan `DSR_ROBOT2`
- `dsr_msgs2`
- `movej`, `movel`, `servol`, `speedl`
- 비동기 ROS 서비스
- ROS 상태 머신

### 비전

- Intel RealSense D435i
- aligned RGB-D
- 일반 USB 2D 웹캠
- OpenCV
- NumPy
- SciPy Rotation
- MediaPipe Tasks Hand Landmarker
- Ultralytics YOLO
- `cv_bridge`
- `sensor_msgs/Image`

### 알고리즘

- Hough Circle Transform
- CLAHE
- Median Blur
- Median과 MAD 이상치 제거
- Hand-eye calibration
- 4×4 homogeneous transform
- EMA low-pass filter
- 주먹 temporal debounce/hysteresis
- 손바닥 크기 기반 monocular relative depth
- XYZ 벡터 이동 제한
- 가속도 제한 스무딩
- 정지 가능 속도 `sqrt(2as)`
- watchdog과 복구 상태 머신

### 그리퍼

- OnRobot RG2
- Modbus TCP
- `pymodbus 2.x`
- busy polling
- grip detected
- safety status bit
- 별도 작업 스레드

### 검증

- ROS dry run
- RViz 및 실제 M0609 시험
- OpenCV 디버그 화면
- ROS Image UI 확인
- `pytest` 스무딩 단위 시험

---

## 13. 최종 평가

프로젝트는 처음의 단순 고정 좌표 이동에서 다음 구조로 발전했다.

```text
센서 보정
 → 좌표계 명시
 → 입력값 검증
 → 다중 프레임 안정화
 → 안전한 분할 경로
 → 그리퍼 상태 확인
 → 속도·가속도 스무딩
 → 손 유실 상태 머신
 → UI 토픽 통합
```

음료 기능은 `YOLO 거친 접근 + OpenCV/Depth 정밀 보정` 방식으로 위치 오차를
줄였고, 손 추종 기능은 `인식 노드와 실행 노드 분리 + 벡터 스무딩 + 복구
상태 머신` 구조로 발전했다.

아직 해결하거나 재검증해야 할 핵심 항목은 다음과 같다.

1. 트레이 접근 좌표와 캔 파지 TCP offset 재확인
2. RG2 폭과 힘을 실제 물체에 맞게 조정
3. follower 주석과 실제 기본값 동기화
4. `cobot_ws`와 `COB2` 코드 버전 동기화
5. 손 추종과 다른 로봇 모션 노드 사이의 단일 제어권 보장
6. `speedl`에서 홈 `amovej`로 넘어가는 모션 상태 전환 강화
7. 필요하면 jerk 제한 스무딩 추가
8. 자동 홈 복귀 경로의 충돌 가능성 검증
9. Doosan bringup 중복 실행 방지
10. RT heartbeat를 위한 네트워크와 `ros2_control_node` 안정성 점검
11. `move_plate`의 busy 검사, 반환값 검사, 안전 경로 추가

현재 구현은 기능 시연과 연구용 프로토타입으로 상당 부분 완성됐지만, 호텔
현장의 무인 반복 운용을 위해서는 제어권 관리, 충돌 회피, 오류 복구, 환경
버전 고정과 실제 작업셀 안전 검증을 추가해야 한다.
