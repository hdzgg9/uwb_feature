# UWB + LiDAR 공통 랜드마크 + GTSAM 다중 로봇 지도 정렬 실험
---

## 실험에서 알고 있는 값과 모르는 값

알고 있다고 가정하는 값:

- UWB 앵커 A0의 절대좌표: `world (0,0)`
- 기준 로봇 `tb3_0`의 초기 pose

미리 입력하지 않는 값:

- `tb3_1`의 실제 초기 위치와 회전
- 원기둥 랜드마크의 world 좌표
- `world -> tb3_1/map` TF

---

## 파일별 설명

### `launch/experiment.launch.py`

1. Gazebo 다중 로봇 월드 실행
2. 로봇별 `robot_state_publisher` 실행
3. 로봇별 `slam_toolbox` 실행
4. 로봇별 UWB 거리 시뮬레이터 실행
5. 로봇별 원기둥 feature detector 실행
6. GTSAM fusion과 alignment waiter 실행
7. `/alignment_ready=true`가 될 때까지 로봇 탐사 대기
8. 정렬 성공 후 `merge_map`, Nav2 controller, frontier 노드 실행


frontier를 실행하지 않고 정렬만 확인하려면
`start_frontier:=false`를 사용

### `config/experiment.yaml`

UWB 노이즈, 원기둥 검출 조건, GTSAM noise model 및 초기화 조건

| 파라미터 | 기본값 | 설명 |
|---|---:|---|
| `anchor_x`, `anchor_y` | `0.0`, `0.0` | 앵커 A0의 world 좌표 |
| `noise_stddev` | `0.05 m` | 모의 UWB 노이즈 |
| `expected_radius` | `0.15 m` | 랜드마크 원기둥 반지름 |
| `radius_tolerance` | `0.055 m` | 반지름 허용 오차 |
| `minimum_common_landmarks` | `3` | 필요한 최소 공통 랜드마크 수 |
| `association_confirmations` | `3` | 동일 매칭 후보 반복 확인 횟수 |
| `minimum_landmark_observations` | `3` | 안정된 랜드마크가 되기 위한 관측 수 |
| `maximum_uwb_initialization_rmse` | `0.20 m` | 초기 후보의 최대 UWB RMSE |
| `keyframe_distance` | `0.12 m` | 위치 keyframe 생성 거리 |
| `keyframe_angle` | `0.10 rad` | 회전 keyframe 생성 각도 |
| `tb3_0_initial_pose` | `[-0.978,1.92,0]` | 기준 로봇의 world 초기 pose |


### `worlds/uwb_feature_world.world`

- 파란색 `anchor_A0`: `(0,0)`에 있는 UWB 앵커
- 주황색 `landmark_0~3`: 반지름 `0.15 m`의 LiDAR 랜드마크
- 외곽 벽과 박스 장애물

앵커 반지름은 `0.07 m`로 일반 랜드마크보다 작기
때문에 circle feature detector의 공통 랜드마크 대상에서 제외

앵커 Gazebo 모델은 위치 표시와 장애물 역할만 
UWB 거리 메시지는 별도의 `uwb_range_sim` 노드가 생성

### `scripts/uwb_range_sim`

실제 UWB 장치 대신 거리 측정값 생성 후 발행

1. `/gazebo/model_states`에서 로봇 위치 읽기
2. 알려진 앵커 좌표까지의 2차원 거리를 계산
3. 가우시안 노이즈 추가
4. `/<robot>/uwb_range`를 발행

```text
d = sqrt((robot_x-anchor_x)^2 + (robot_y-anchor_y)^2) + noise
```
GTSAM에는 Gazebo의 `tb3_1` pose가 전달되지 않고 UWB 거리만 전달

실물에서는 이 노드를 실제 UWB driver로 교체

### `scripts/circle_feature_detector`

`/<robot>/scan`에서 원기둥을 검출합니다.

처리 과정:

1. LaserScan을 XY 점군으로 변환
2. 가까운 연속 점들을 클러스터링
3. 클러스터별 최소제곱 원 피팅
4. 반지름과 fit error 조건 검사
5. 원 중심의 range와 bearing 계산
6. `/<robot>/features` 발행

```text
range   = sqrt(center_x^2 + center_y^2)
bearing = atan2(center_y, center_x)
```

### `msg/CircleFeature.msg`

원기둥 한 개의 검출 결과

| 필드 | 설명 |
|---|---|
| `track_id` | detector 내부의 scan 간 추적 ID |
| `landmark_id` | 공통 ID용 필드, detector 출력에서는 `-1` |
| `center` | LiDAR 좌표계 기준 원 중심 |
| `radius` | 피팅된 반지름 |
| `range` | 로봇에서 원 중심까지 거리 |
| `bearing` | 로봇 전방 기준 방위각 |
| `fit_error` | 원 피팅 오차 |

### `msg/CircleFeatureArray.msg`

한 번의 scan에서 검출된 여러 `CircleFeature`와 scan header를 전달

### `scripts/gtsam_fusion`

초기 상대 정렬과 factor graph 최적화를 수행하는 핵심 노드입니다.

구독:

```text
/tb3_0/odom             /tb3_1/odom
/tb3_0/uwb_range        /tb3_1/uwb_range
/tb3_0/features         /tb3_1/features
```

초기 정렬 과정:

1. 로봇별 odom 시작 좌표계에서 랜드마크 배치를 누적
2. 반복 검출된 feature만 안정된 랜드마크로 선택
3. 두 로봇의 랜드마크 사이 거리 배치를 비교
4. 상대 이동·회전 후보 계산
5. 나머지 랜드마크로 consensus 검사
6. 후보를 world에 배치하고 예상 UWB 거리 계산
7. 실제 UWB 측정과의 RMSE 검사
8. 동일 대응관계가 여러 scan 구간에서 확인되면 graph 초기화

사용하는 GTSAM factor:

| Factor | 입력 | 역할 |
|---|---|---|
| `PriorFactorPose2` | `tb3_0` 초기 pose | world 기준 고정 |
| `PriorFactorPoint2` | 앵커 `(0,0)` | 앵커 좌표 고정 |
| `BetweenFactorPose2` | odometry 변화량 | 연속 pose 연결 |
| `RangeFactor2D` | UWB 거리 | pose-앵커 거리 제약 |
| `BearingRangeFactor2D` | LiDAR 거리·방향 | pose-landmark 연결 |

출력:

```text
/tb3_0/gtsam_pose
/tb3_1/gtsam_pose
/alignment_ready
/tf_static의 world -> tb3_0/map
/tf_static의 world -> tb3_1/map
```

`/tb3_x/gtsam_pose`는 계속 갱신되지만 map TF는 정렬 성공 시 한 번만
고정

### `scripts/alignment_waiter`

`/alignment_ready=true` 성공하여 이 노드가 정상
종료된 후에 `merge_map`, Nav2 controller 및 frontier가 시작

### `scripts/scan_wander`


### `urdf/turtlebot3_waffle_pi_tf.urdf`

`robot_state_publisher`가 로봇 link TF를 발행할 때 사용 로봇별
`frame_prefix`를 적용하여 `tb3_0/base_link`, `tb3_1/base_link`처럼
프레임 이름 충돌을 방지

---


## 빌드와 실행

GTSAM 확인:

```bash
python3 -c "import gtsam; print('GTSAM import OK')"
```

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select uwb_feature_gtsam_sim merge_map frontier_ws
source install/setup.bash
ros2 launch uwb_feature_gtsam_sim experiment.launch.py
```

frontier를 실행하지 않고 정렬만 시험:

```bash
ros2 launch uwb_feature_gtsam_sim experiment.launch.py start_frontier:=false
```

Gazebo GUI 없이 실행:

```bash
ros2 launch uwb_feature_gtsam_sim experiment.launch.py gui:=false
```

---

## 정상 동작 확인

### UWB 및 feature

```bash
ros2 topic echo /tb3_0/uwb_range --once
ros2 topic echo /tb3_1/uwb_range --once
ros2 topic echo /tb3_0/features --once
ros2 topic echo /tb3_1/features --once
```

정상 초기화 로그:

```text
CALIBRATING: stable landmarks tb3_0=4, tb3_1=4;
scans=(...), UWB samples=(100,100)

Common-landmark candidate 1/3: ...
Common-landmark candidate 2/3: ...
JOINT GRAPH INITIALIZED ...
INITIAL MAP ALIGNMENT FROZEN ...
```

정렬 상태 확인:

```bash
ros2 topic echo /alignment_ready --once
```

정상 결과는 `data: true`

Static TF 확인:

```bash
ros2 topic echo /tf_static --once
ros2 run tf2_ros tf2_echo world tb3_0/map
ros2 run tf2_ros tf2_echo world tb3_1/map
```

지도 확인:

```bash
ros2 topic echo /tb3_0/map --once
ros2 topic echo /tb3_1/map --once
ros2 topic echo /merge_map --once
```

RViz Fixed Frame은 `world`로 설정

---

### RViz에서 `Frame [world] does not exist`

초기 정렬 전에는 `world -> map`을 발행하지 않으므로 정상

## 실제 실험

`uwb_range_sim` 대신 실제 UWB driver가 다음 토픽을 발행하도록

```text
/tb3_0/uwb_range    sensor_msgs/msg/Range
/tb3_1/uwb_range    sensor_msgs/msg/Range
```

