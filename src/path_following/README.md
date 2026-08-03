# path_following — Stanley 경로 추종 + FGM 회피 (Jetson 실차)

F1TENTH Ajou 젯슨 실차용 주행 스택. Cartographer **map** 프레임 CSV를 따라가며, 정적 장애물 회피(FGM + local planner)를 수행한다.

**워크스페이스:** `/home/nvidia/f1tenth_ajou`

---

## 노드 구성

```
/static_obstacles  ← static_obstacle_node  (/scan)
/fgm_target        ← fgm_node              (/scan)
/strategy/*        ← drive_strategy_node   (CSV + TF)
/local_path        ← local_planner_node    (CSV + 장애물 + FGM)
/drive             ← stanley_waypoint_follow_node
                     ↓
                   control_node            (VESC + ESP32, 별도 터미널 권장)
```

| 실행 파일 | 역할 |
|-----------|------|
| `static_obstacle_node` | LiDAR 클러스터 → `/static_obstacles` |
| `fgm_node` | 갭 알고리즘 → `/fgm_target` |
| `drive_strategy_node` | 곡선/직선/장애 거리 → 속도 배율 제안 |
| `local_planner_node` | CSV ↔ 회피 상태머신 → `/local_path` |
| `stanley_waypoint_follow_node` | Stanley 추종 → `/drive` |
| `control_node` | `/drive` → 모터·조향 (Space=ESTOP) |

**튜닝:** 각 `path_following/*.py` 상단 `CFG` 딕셔너리.

**CSV:** 노드 `CFG["csv_path"]`가 비어 있으면 `config/raceline.csv` → `config/centerline.csv` 순으로 자동 탐색 (`track_sliding.resolve_csv_path`).

---

## 0. 환경 (매 터미널)

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/f1tenth_ajou/install/setup.bash
```

---

## 1. 빌드

코드·런치·CSV 수정 후:

```bash
cd /home/nvidia/f1tenth_ajou
source /opt/ros/humble/setup.bash
colcon build --packages-select path_following localization_layer
source install/setup.bash
```

`path_following`만 바꿨을 때:

```bash
colcon build --packages-select path_following
source install/setup.bash
```

설치된 실행 파일 확인:

```bash
ros2 pkg executables path_following
```

---

## 2. 맵 → 센터라인 → 레이싱라인 (스크립트)

**의존성:** `numpy`, `scipy`, `PyYAML`, `Pillow`, `scikit-image`

```bash
pip3 install numpy scipy pyyaml pillow scikit-image
```

### 2.1 센터라인 추출

맵은 **PNG가 아니라 같은 이름의 `*_rosmap.yaml`** 을 넘긴다.

```bash
cd /home/nvidia/f1tenth_ajou/src/path_following/scripts
python3 extract_centerline_from_map.py
# 기본 맵: maps/cartographer_map_20260711_200005.yaml


# 다른 맵:
python3 extract_centerline_from_map.py \
  --map /home/nvidia/f1tenth_ajou/maps/<맵이름>_rosmap.yaml \
  --out ../config/centerline.csv
```

주요 옵션:

| 옵션 | 설명 |
|------|------|
| `--invert-free auto` | 밝/어두 도로 자동 판별 (Cartographer 맵은 보통 `auto`) |
| `--resample-step-m 0.05` | 출력 점 간격 [m] |
| `--out ../config/centerline.csv` | 기본 출력 경로 |

### 2.2 레이싱라인 생성

```bash
python3 generate_raceline_from_centerline.py

# 또는 경로 직접 지정:
python3 generate_raceline_from_centerline.py \
  --centerline ../config/centerline.csv \
  --map /home/nvidia/f1tenth_ajou/maps/cartographer_map_20260711_200005.yaml \
  --out ../config/raceline.csv
```

생성 후 빌드해야 노드가 install 쪽 CSV를 읽는다:

```bash
cd /home/nvidia/f1tenth_ajou
colcon build --packages-select path_following
source install/setup.bash
```

### 2.3 맵 ↔ CSV ↔ pbstream 일치 (필수)

| 항목 | 반드시 |
|------|--------|
| `extract` / `generate` 의 `--map` | 같은 rosmap YAML |
| 로컬 `pbstream_filename` | **위 맵과 쌍을 이루는 `.pbstream`** |
| `config/raceline.csv` | 위 YAML에서 뽑은 경로 |

예: CSV를 `200005` 맵으로 만들었다면 로컬도 `200005.pbstream`을 쓴다.

---

## 3. 매핑 (새 맵 만들기)

`localization_layer` 패키지. 센서(LiDAR·IMU·TF) 포함.

```bash
cd /home/nvidia/f1tenth_ajou
source install/setup.bash

ros2 launch localization_layer cartographer_mapping_launch.py
```

- 맵 저장: `~/f1tenth_ajou/maps/` (`cartographer_map_*.pbstream`, `*_rosmap.yaml/png`)
- 종료 시 자동 저장 (`Ctrl+C`, `save_on_shutdown:=true` 기본)

센서를 이미 띄워 둔 경우:

```bash
ros2 launch localization_layer cartographer_mapping_launch.py enable_sensor_bringup:=false
```

매핑이 끝나면 **§2 스크립트**로 `centerline.csv` / `raceline.csv`를 다시 만든다.

---

## 4. 로컬라이제이션 (실차 주행 전)

LiDAR 네트워크 설정 → 센서 bringup → Cartographer pure localization.  
TF: **`map` → `base_link`**, 토픽: **`/scan`**.

```bash
ros2 launch localization_layer cartographer_localization_launch.py
```

기본 pbstream (런치 파일 기준):

`/home/nvidia/f1tenth_ajou/maps/cartographer_map_20260711_200005.pbstream`

**현재 `config/raceline.csv`는 `200005` 맵 기준** (launch 기본 pbstream과 동일):

```bash
ros2 launch localization_layer cartographer_localization_launch.py \
  pbstream_filename:=/home/nvidia/f1tenth_ajou/maps/cartographer_map_20260711_200005.pbstream
```

자주 쓰는 옵션:

| 인자 | 기본 | 설명 |
|------|------|------|
| `pbstream_filename` | `200005.pbstream` | 로드할 맵 |
| `enable_sensor_bringup` | `true` | LiDAR·IMU·TF 자동 기동 |
| `cartographer_startup_delay_sec` | `6.0` | 센서 워밍업 후 Cartographer 시작 |
| `wait_for_rviz_initial_pose` | `true` | RViz 2D Pose Estimate 대기 |
| `use_rviz` | `false` | Jetson에서 RViz 띄우기 |

RViz (데스크톱, launch와 **같은** `setup.bash`):

```bash
ros2 run localization_layer run_localization_rviz.sh
```

확인:

```bash
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo map base_link
```

---

## 5. 주행 스택 (Stanley + 회피)

### 런치 파일

| 파일 | 패키지 | 설명 |
|------|--------|------|
| `launch/path_follow_stanley_launch.py` | `path_following` | 주행 5노드 (+ 선택 `control_node`) |

```bash
ros2 launch path_following path_follow_stanley_launch.py
```

런치 인자:

| 인자 | 기본 | 설명 |
|------|------|------|
| `enable_vehicle_control` | `false` | `true`면 런치에 `control_node` 포함 |
| `status_log_hz` | `2.0` | Stanley STATUS 로그 (0.5초마다 1줄) |
| `verbose_logs` | `false` | local_planner 상세 로그 |

상세 로그:

```bash
ros2 launch path_following path_follow_stanley_launch.py verbose_logs:=true
```

---

## 6. 실차 주행 — 터미널 순서

### 터미널 1 — 로컬라이제이션 + 센서

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/f1tenth_ajou/install/setup.bash

ros2 launch localization_layer cartographer_localization_launch.py \
  pbstream_filename:=/home/nvidia/f1tenth_ajou/maps/cartographer_map_20260711_200005.pbstream
```

RViz에서 **2D Pose Estimate**로 차량 위치를 맞춘다 (`wait_for_rviz_initial_pose:=true`일 때).

### 터미널 2 — 주행 알고리즘

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/f1tenth_ajou/install/setup.bash

ros2 launch path_following path_follow_stanley_launch.py
```

### 터미널 3 — 모터·조향 (권장)

키보드 **Space = ESTOP**, **R = 리셋**. 별도 터미널에서 띄우는 것을 권장한다.

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/f1tenth_ajou/install/setup.bash

ros2 run path_following control_node
```

한 터미널에 합치려면:

```bash
ros2 launch path_following path_follow_stanley_launch.py enable_vehicle_control:=true
```

### 터미널 4 — 디버그 모니터 (튜닝 참고)

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/f1tenth_ajou/install/setup.bash

ros2 run path_following drive_monitor
```

속도·조향·CSV/회피/REJOIN 모드·LiDAR·장애물을 2Hz로 갱신 표시.

---

## 7. 시뮬 (참고)

Gym 브릿지 + 주행 (하드웨어 없음):

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
ros2 launch path_following path_follow_stanley_launch.py enable_vehicle_control:=false
```

시뮬 TF 프레임이 다르면 노드 `CFG`의 `map_frame` / `base_frame` / `laser_frame`을 맞춘다.

---

## 8. 실행 전 체크리스트

- [ ] `colcon build` 후 `source install/setup.bash`
- [ ] `config/raceline.csv` 존재 (또는 `centerline.csv`)
- [ ] **pbstream ↔ raceline CSV ↔ rosmap YAML** 같은 맵 세트
- [ ] `/scan` 발행 (`ros2 topic hz /scan`)
- [ ] `map` → `base_link` TF (`tf2_echo map base_link`)
- [ ] LiDAR 네트워크 (Jetson `192.168.11.3` ↔ LiDAR `192.168.11.2`)
- [ ] VESC `/dev/ttyACM0`, ESP `/dev/ttyTHS1` 연결
- [ ] RC CH5: 수동 ↔ 자율 전환 확인

주행 중 확인:

```bash
ros2 topic echo /drive --once
ros2 topic hz /local_path
```

Stanley가 CSV를 못 찾으면 시작 시 `csv_path is required` / `FileNotFoundError` 로 종료한다.

---

## 9. 디렉터리

```
path_following/
├── README.md                 ← 이 파일
├── config/
│   ├── centerline.csv        ← extract 스크립트 출력
│   └── raceline.csv          ← generate 출력 (노드 기본 경로)
├── launch/
│   └── path_follow_stanley_launch.py
├── scripts/
│   ├── extract_centerline_from_map.py
│   └── generate_raceline_from_centerline.py
└── path_following/           ← ROS 노드 (*.py, CFG 튜닝)
```

관련 패키지 (`localization_layer`):

```
localization_layer/launch/
├── cartographer_mapping_launch.py       ← 매핑
├── cartographer_localization_launch.py  ← 실차 로컬
└── mapping_sensor_bringup_launch.py     ← (로컬/매핑 내부) 센서
```

---

## 11. 실차 디버그 모니터 (튜닝용)

주행 중 **별도 터미널**에서 속도·조향·모드·LiDAR·장애물을 한 화면에 표시.

```bash


# control_node + 주행 스택이 떠 있는 상태에서

ros2 run path_following drive_monitor

# 또는
source install/setup.bash

```

**표시 항목**

| 섹션 | 내용 |
|------|------|
| 모드 | CH5 수동/자율, planner GLOBAL/AVOID/REJOIN, Stanley CSV/LOCAL |
| 속도 | `/drive` 명령, `/odom` 또는 TF 추정, VESC duty |
| 조향 | `/drive` rad/deg, ESP `S:` 전송값 |
| LiDAR | `/scan` Hz, 전방 최소거리, `/static_obstacles` 개수·최근접 |
| FGM | `/fgm_target` 거리·방향 |

**필요 토픽**

- `/vehicle/telemetry` ← `control_node` (duty, RC, AUTO/MANUAL)
- `/planner/mode` ← `local_planner_node` (GLOBAL/AVOID/REJOIN)
- `/drive`, `/scan`, `/static_obstacles`, `/fgm_target`, `/planner_path_override_active`

코드 수정 후: `colcon build --packages-select path_following`

---

## 10. 튜닝 요약 (현재 CFG 기본값)

| 노드 | 주요 파라미터 |
|------|----------------|
| `stanley_waypoint_follow_node` | `max_steering_angle` ±40°, `stanley_k` 1.5, `max_drive_speed` 1.5 |
| `local_planner_node` | `avoid_on_m` 1.8, `avoid_pass_rear_x_m` -0.35, 직진 leg 30×0.15 m |
| `drive_strategy_node` | 직선 `speed_straight_mul` 2.0, 곡선 `speed_curve_mul` 0.5 |
| `fgm_node` | 목표점 = `target_lead_time_s` 0.55 × 속도 (0.9~3.0 m), `bubble_radius_m` 0.30, `gap_edge_inset_deg` 3 |
| `static_obstacle_node` | `max_obstacle_size_m` 0.6, `min_obstacle_size_m` 0.1 |
| `control_node` | `max_duty` 0.20, `invert_steer` false |

### 조향 부호 통일 (실차 / ESP)

서보 `LEFT_ANGLE`/`RIGHT_ANGLE` 이름과 **실제 차량 좌우가 반대**다.
(MANUAL에서 CH1 1000→`RIGHT_ANGLE`이어야 좌로 꺾임.)

| 계층 | 규약 |
|------|------|
| `/drive`, ESP `S:` | **+ = 좌**, **- = 우** (`S:+1`→140°=좌, `S:-1`→40°=우) |
| `invert_steer` | `false` |
| map / laser / FGM | ROS TF (**+x 전방, +y 좌**) |

프레임 (실차): `map` / `base_link` / `laser`


빌드
cd /home/nvidia/f1tenth_ajou
source /opt/ros/humble/setup.bash
colcon build --packages-select path_following localization_layer
source install/setup.bash


센터
cd /home/nvidia/f1tenth_ajou/src/path_following/scripts
python3 extract_centerline_from_map.py

레이싱
python3 generate_raceline_from_centerline.py


로컬
source /opt/ros/humble/setup.bash
source /home/nvidia/f1tenth_ajou/install/setup.bash
ros2 launch localization_layer cartographer_localization_launch.py


주행
source /opt/ros/humble/setup.bash
source /home/nvidia/f1tenth_ajou/install/setup.bash
ros2 launch path_following path_follow_stanley_launch.py
ros2 launch path_following path_follow_static_dynamic_avoid_launch.py



컨트롤
source /opt/ros/humble/setup.bash
source /home/nvidia/f1tenth_ajou/install/setup.bash
ros2 run path_following control_node


디버깅
source /opt/ros/humble/setup.bash
source /home/nvidia/f1tenth_ajou/install/setup.bash
ros2 run path_following drive_monitor


젯슨 실제 확인
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765


ws://192.168.137.20:8765

로거 노드
source /opt/ros/humble/setup.bash
source /home/nvidia/f1tenth_ajou/install/setup.bash
ros2 launch path_following csv_logger_launch.py


맵핑
cd /home/nvidia/f1tenth_ajou
source install/setup.bash
ros2 launch localization_layer cartographer_mapping_launch.py


Stanley 제어 텔레메트리는 `/stanley/debug` (`std_msgs/Float64MultiArray`)로
각 제어 주기마다 한 메시지로 발행된다. 배열 순서와 단위는 다음과 같다.

0. `cross_track_error_m` [m]
1. `heading_error_rad` [rad]
2. `heading_term_rad` [rad]
3. `cross_track_term_rad` [rad]
4. `stanley_steering_sum_rad` [rad, saturation 전]
5. `raw_steering_cmd_rad` [rad, Stanley saturation 후]
6. `filtered_or_limited_steering_cmd_rad` [rad, smoothing/rate limit 후]
7. `stanley_speed_mps` [m/s, Stanley 분모에 사용]
8. `closest_path_index` [현재 선택된 경로 배열의 segment index]


직진 노드
source /opt/ros/humble/setup.bash
source /home/nvidia/f1tenth_ajou/install/setup.bash
ros2 run path_following straight_drive_publisher



git 업로드

팀 레포
cd /home/nvidia/f1tenth_ajou
git add src/
git commit -m "작업 내용"
git push roboracer-ajou

전체 선택
cd /home/nvidia/f1tenth_ajou
git add .
git commit -m "오늘 작업 전체 백업"
git push roboracer 

roboracer이건 내 레포 origin이건 팀 레포

일부만 업로드
cd /home/nvidia/f1tenth_ajou
git add src/path_following/path_following/control_node.py
git commit -m "fix: control_node 수정"
git push roboracer

일부만 빼고 싶을 때
cd /home/nvidia/f1tenth_ajou
git add .
git reset maps/
git reset *.csv

코드만 수정
cd /home/nvidia/f1tenth_ajou
git add src/ README.md .gitignore
git commit -m "path_following 수정"
git push roboracer
