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
| `drive_strategy_node` | 곡선/직선/장애 거리 → 속도 배율 제안 (**현재 미구현**) |
| `emergency_brake_node` | `/scan` TTC → `/emergency_brake`. 플래너와 독립된 안전 계층 → 5.3 |
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

스켈레톤에서 인필드를 감싸는 폐루프를 뽑은 뒤, 거리변환 능선(벽-벽 중앙)으로
당기면서 스무딩한다. 이동할 때마다 클리어런스를 검사하므로 벽을 넘지 않는다.

주요 옵션:

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--resample-step-m` | `0.05` | 출력 점 간격 [m] |
| `--min-clear-m` | `0.20` | 벽에서 유지할 최소 거리 [m] |
| `--min-radius-m` | `0.9` | 최소 회전반경 [m]. 더 급한 코너만 국소적으로 편다 |
| `--smooth-iters` | `120` | 능선 스무딩 반복. 키우면 더 부드럽다 |
| `--invert-free` | off | 어두운 픽셀을 도로로 해석 (기본은 ROS 규약대로 밝은 쪽) |
| `--out` | `../config/centerline.csv` | 출력 경로 |

실행하면 꺾임(`turn |Δθ|`), 클리어런스, `center_ratio`, 벽 관통/자기교차 수가
출력된다. **`wall_crossings`·`self_intersections` 가 0이 아니면 쓰면 안 된다.**

### 2.2 레이싱라인 생성

```bash
python3 generate_raceline_from_centerline.py

# 또는 경로 직접 지정:
python3 generate_raceline_from_centerline.py \
  --centerline ../config/centerline.csv \
  --map /home/nvidia/f1tenth_ajou/maps/cartographer_map_20260711_200005.yaml \
  --out ../config/raceline.csv
```

기본 방식은 **최소곡률 최적화**다. 코너 속도가 `v = √(a_lat/κ)` 이므로
곡률 제곱합을 줄이는 게 곧 랩타임 단축이다.

라인을 `x_i = p_i + α_i·n_i` (센터라인 + 법선방향 오프셋)로 두면 2차차분이
`α` 의 아핀함수가 되어 아래가 **볼록 QP** 가 된다.

```
min  Σ ‖x_{i-1} − 2x_i + x_{i+1}‖²      s.t.  lo_i ≤ α_i ≤ hi_i
```

`lo/hi` 는 각 점에서 법선으로 벽까지 레이캐스팅한 거리에서 `--margin-m` 을 뺀 값이라
**해가 트랙을 벗어날 수 없다.** 희소 능동집합법으로 전역해를 구하고(QP 1회 ≈ 0.05초),
구한 라인을 다시 기준선으로 삼아 법선·경계를 갱신해 재수렴한다.

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--method` | `mincurv` | `mincurv`=최소곡률, `oio`=구 휴리스틱 Out-In-Out |
| `--margin-m` | `0.35` | 벽에서 뗄 거리 [m] = 차량 반폭 + 여유. **최적해는 이 경계까지 붙는다** |
| `--iterations` | `2` | 재선형화 반복 |
| `--w-length` | `0.0` | >0 이면 최단경로 쪽으로 살짝 당김 |
| `--min-radius-m` | `0.9` | 최소 회전반경 [m] |
| `--min-clear-m` | `0.30` | 최종 검증용 최소 벽 거리 [m] |
| `--a-lat` / `--v-max` | `6.0` / `8.0` | 속도 프로파일용 (경로 모양에는 영향 없음) → 2.3 |

무섭게 붙는다 싶으면 `--margin-m` 만 키우면 된다. 실행하면 센터라인 대비
예상 랩타임이 같이 찍히므로 설정을 바꿔가며 비교할 수 있다.

```
raceline: 761 pts, length=38.03 m (centerline 40.86 m)
curvature max=0.734 1/m (R_min=1.36 m), turn |Δθ| max=2.84°
clearance min=0.375 m median=0.699 m
est. lap=7.24 s (centerline 9.16 s, +21.0%), v min=2.86 mean=5.78 m/s
```

**주의:** 최소곡률은 랩타임 최적의 표준 근사이지 진짜 최소시간 해는 아니다.
긴 직선 앞 코너를 탈출속도를 위해 희생하는 식의 전역 트레이드오프는 반영되지 않는다.
그걸 하려면 마찰원·가감속을 포함한 최소시간 최적화가 필요하다.

이 트랙 실측 비교 (`--a-lat 6.0`):

| 라인 | 길이 | 예상 랩 | R_min |
|------|------|---------|-------|
| 센터라인 | 40.86 m | 9.16 s | 1.09 m |
| O-I-O (`--method oio`) | 41.84 m | 10.21 s | 1.08 m |
| **최소곡률 (기본)** | **38.03 m** | **7.24 s** | **1.36 m** |

O-I-O 휴리스틱은 고정 비율로 코너마다 독립적으로 벌렸다 붙였다 하는 방식이라
직선 구간에서 오히려 불필요하게 휘어 센터라인보다도 느리다. 비교용으로만 남겨뒀다.

생성 후 빌드해야 노드가 install 쪽 CSV를 읽는다:

```bash
cd /home/nvidia/f1tenth_ajou
colcon build --packages-select path_following
source install/setup.bash
```

### 2.3 속도 프로파일 (CSV `v` 열)

두 스크립트 모두 기본으로 `x,y,v` 3열을 쓴다 (`--no-speed` 면 `x,y`).
`scripts/speed_profile.py` 가 공용 구현이다.

주행 중 "코너 감지" 는 하지 않는다. 오프라인에서 경로 곡률 κ 와 차량 가감속
한계만으로 전 구간 속도를 미리 정한다.

```
1) 곡률 한계   v_i = min(v_max, √(a_lat / κ_i))
2) 역방향 패스 v_i = min(v_i, √(v_{i+1}² + 2·a_brake·ds))   ← 브레이킹 포인트
3) 정방향 패스 v_i = min(v_i, √(v_{i-1}² + 2·a_accel·ds))   ← 코너 탈출 가속
```

1) 이 "U턴만 느리게, 완만한 곡선은 직선처럼" 을 자동으로 만든다. 반경이 크면
κ 가 작아 `v_max` 에 걸리기 때문에 **"90도 이상만 감속" 같은 분류 규칙이 필요 없다.**
2) 는 코너에 닿기 전부터 속도를 깎아 감속 시작점을 알아서 앞당긴다.

#### 차량 물리 파라미터

`scripts/speed_profile.py` 상단 `VEHICLE` dict 가 기본값이고, CLI 로 덮어쓴다.
**현재 값은 실측 전 임시값**이라 실행하면 경고가 뜬다. 측정 후 dict 를 채우고
`measured=True` 로 바꾸면 경고가 사라진다.

| 옵션 | 측정 방법 |
|------|-----------|
| `--a-lat` | 횡가속 한계 [m/s²]. 반경 R 로 돌다 미끄러지는 속도 v → `a = v²/R`. **제일 중요** |
| `--a-brake` | 감속 한계 [m/s²]. 직선 풀브레이크 정지거리 d → `a = v²/(2d)` |
| `--a-accel` | 가속 한계 [m/s²]. 0→v 도달 거리 d → `a = v²/(2d)` |
| `--v-max` | 직선 최고속도 캡 [m/s]. 모터·기어비 또는 대회 규정 |
| `--safety-factor` | 위 세 가속도에 곱하는 안전계수. 실차 검증 전 `0.6~0.7` |

#### 기준속도로 전체 스케일하기

`--v-max` 를 낮추면 **직선만** 느려진다. 코너 속도 `√(a_lat/κ)` 는 접지력이
정하는 값이라 최고속도와 무관하기 때문이다. 알고리즘 경향만 보려고 전 구간을
느리게 하고 싶으면 `--v-ref` 를 쓴다.

`VEHICLE["v_ref_mps"]` 가 기본값이고 CLI 로 덮어쓴다. **보통 여기만 만진다.**

```python
"v_ref_mps": 2.0,        # 기준 최고속도 [m/s]. 0 = 비활성
```

```bash
# "최고속도 5 m/s 기준으로 전 구간" — 코너는 물리 비율대로 자동 축소
python3 generate_raceline_from_centerline.py --v-ref 5.0
```

프로파일 최댓값이 `v_ref` 가 되도록 배율을 역산해 전체에 곱한다. 직선:코너
비율은 물리값 그대로 유지된다. 균일 축소는 가·감속 제약을 항상 만족하므로 안전하다.

```
v_ref=2.00 m/s → speed_scale=0.250 (물리 최고속 8.00 m/s 기준 자동 계산)
speed profile: v min=0.73 max=2.00 mean=1.46 m/s (감속구간 34% of lap)
est. lap=29.71 s (centerline 37.30 s, +20.3%)
```

배율을 직접 주려면 `--speed-scale 0.25` 도 같다. `--v-min` 은 완전 정지를 막는
하한인데, 물리 한계보다 빠른 지령이 되므로 많이 걸리면 경고가 뜬다.

**현재 `config/*.csv` 는 `v_ref_mps=2.0` (기본값) 으로 생성돼 있다.** 실측값을
넣은 뒤 다시 뽑을 것. 바꿨으면 **센터라인·레이스라인 둘 다** 다시 뽑고 빌드한다.

```bash
cd src/path_following/scripts
python3 extract_centerline_from_map.py
python3 generate_raceline_from_centerline.py
cd /home/nvidia/f1tenth_ajou && colcon build --packages-select path_following
```

### 2.4 맵 ↔ CSV ↔ pbstream 일치 (필수)

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
| `track` | `""` | 주행 라인 `raceline` \| `centerline` \| `auto`. 빈 값이면 `DEFAULT_TRACK` |
| `enable_vehicle_control` | `false` | `true`면 런치에 `control_node` 포함 |
| `status_log_hz` | `2.0` | Stanley STATUS 로그 (0.5초마다 1줄) |
| `verbose_logs` | `false` | local_planner 상세 로그 |

상세 로그:

```bash
ros2 launch path_following path_follow_stanley_launch.py verbose_logs:=true
```

### 5.1 레이싱라인 ↔ 센터라인 전환

`local_planner_node` 와 `stanley_waypoint_follow_node` 가 **같은 CSV** 를 봐야 한다.
한쪽만 바꾸면 플래너는 센터라인 기준으로 회피 코리도어를 만드는데 추종은
레이싱라인을 따라가서 서로 싸운다. 그래서 `track` 인자 하나가 두 노드에 동시에 들어간다.

```bash
# 센터라인 (벽-벽 중앙)
ros2 launch path_following path_follow_static_dynamic_avoid_launch.py track:=centerline

# 레이싱라인 (Out-In-Out)
ros2 launch path_following path_follow_static_dynamic_avoid_launch.py track:=raceline
```

빌드 없이 즉시 적용된다. 세 런치 파일(`path_follow_stanley` / `path_follow_static_avoid` /
`path_follow_static_dynamic_avoid`) 모두 지원한다.

기본값을 아예 바꾸려면 `path_following/track_sliding.py` 의 상수 하나만 고친다.

```18:18:src/path_following/path_following/track_sliding.py
DEFAULT_TRACK = "raceline"
```

어느 라인이 물렸는지 확인하려면 런타임에 파라미터를 조회한다. 두 값이 같아야 정상이다.

```bash
ros2 param get /stanley_waypoint_follow_node csv_path
ros2 param get /local_planner_node csv_path
```

노드 시작 로그에도 대괄호로 찍히지만, 런치는 `--log-level warn` 으로 띄우기 때문에
`ros2 run` 으로 직접 실행할 때만 보인다.

```
Stanley waypoint follower | track=[centerline.csv] CSV=/.../config/centerline.csv
CSV track loaded: [centerline.csv] /.../config/centerline.csv (817 pts)
```

`ros2 run` 으로 노드를 따로 띄울 때는 파라미터로 준다. **두 노드 모두** 같은 값으로.

```bash
ros2 run path_following stanley_waypoint_follow_node --ros-args -p track:=centerline
ros2 run path_following local_planner_node          --ros-args -p track:=centerline
```

`config/` 밖의 CSV 를 쓰려면 `track` 대신 `csv_path` 에 절대경로를 준다 (`csv_path` 가 우선).

### 5.2 구간별 속도 (CSV `v` 열)

속도는 주행 중에 코너를 "감지" 해서 줄이는 게 아니라, **CSV 3번째 열에 미리
박아둔 값**을 그대로 따라간다. 감속·가속 지점이 이미 경로에 포함돼 있으므로
런타임에는 조회만 한다.

```
raceline.csv ──(v열)──> stanley ──/drive.speed──> control_node ──FF+PI──> VESC duty
                              ▲                        ▲
                    /planner/speed_scale          /emergency_brake
                    (회피·선감속, ≤1.0)           (역토크, 최우선)
```

우선순위는 **아래일수록 이긴다.** CSV 속도는 장애물이 없을 때의 상한일 뿐이고,
그보다 빠른 명령은 나가지 않는다.

1. CSV `v` (글로벌 패스, 가장 후순위)
2. `/planner/speed_scale` — 정적/동적 장애 선감속·회피 조향 한계. `min(1, v_avoid/v_csv)`
3. `/emergency_brake` — AEB. 속도 PI 를 무시하고 역토크
4. ESTOP (RC CH6 / 키보드) — latch, 사람이 리셋

그래서 직선 CSV 가 5 m/s 여도 4 m 앞 콘이 있으면 명령은 ~3 m/s 로 내려가고,
1 m 안 돌발이면 AEB 가 duty 를 가져간다.

| 노드 | 하는 일 |
|------|---------|
| `stanley_waypoint_follow_node` | 현재 투영된 웨이포인트의 `v` 를 `/drive.speed` 로 발행 |
| `control_node` | `use_drive_speed_command=True` 면 그 값을 목표로 FF+PI 추종 |

예전에는 Stanley 가 `speed=0.0` 만 보내고 control_node 가 `target_speed_mps`
정속으로 달렸다. 그래서 "속도 명령이 안 먹는" 것처럼 보였다.

**전 구간 정속으로 되돌리려면** `control_node.py` 의 `use_drive_speed_command`
를 `False` 로 두면 `target_speed_mps` 정속이 된다 (구동계 튜닝할 때 편하다).

안전장치는 그대로다. `/drive` 가 `cmd_timeout_sec`(0.25s) 이상 끊기면 duty·조향
모두 0 으로 떨어지고, 목표 속도는 `max_target_speed_mps` 로 클램프된다.

#### 재생성 없이 현장에서 줄이기

`stanley_waypoint_follow_node` 파라미터:

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `speed_scale` | `1.0` | CSV 값 전체 배율. 급하게 느리게 할 때 |
| `speed_max_mps` | `0.0` | 하드 상한 [m/s]. 0 = 제한 없음 |
| `speed_lookahead_m` | `0.3` | 앞 구간 최솟값을 취해 제어 지연 흡수. 감속이 살짝 앞당겨진다 |
| `speed_from_csv` | `True` | `False` 면 `speed_fallback_mps` 정속 |
| `speed_fallback_mps` | `0.0` | v 열 없는 구형 CSV 일 때 쓸 값. **0 이면 차가 안 움직인다** |

```bash
ros2 launch path_following path_follow_stanley_launch.py track:=raceline
ros2 param set /stanley_waypoint_follow_node speed_scale 0.5   # 주행 중 절반으로
```

시작 로그로 어디서 속도가 왔는지 확인할 수 있다.

```
drive=/drive (steer + speed), speed_src=csv v[0.73~2.00]m/s scale=1.00 cap=none
```

`speed_src=fallback(csv has no v column)` 이 보이면 CSV 를 다시 뽑아야 한다.

#### 회피·추월 전략과의 관계

장애물이나 다른 차량처럼 **CSV 에 없는 변수**는 런타임 배율로 처리한다.
Stanley 가 `/planner/speed_scale` (`std_msgs/Float64`) 를 구독해 CSV 속도에 곱한다.

```
최종 목표속도 = CSV v × speed_scale(파라미터) × /planner/speed_scale(전략)
```

**감속만 반영된다** (1.0 초과는 1.0 으로 잘림). CSV 값이 이미 접지력 한계라
그 위로 올리면 코너에서 그립을 잃기 때문이다. `local_planner_node` 가
`/strategy/speed_multiplier` 를 받아 이 토픽으로 중계하는데, 지금은
`strategy_bridge_enable=False` 로 꺼져 있다.

### 5.3 비상 제동 (AEB)

`emergency_brake_node` 는 **플래너와 독립된 최후 안전 계층**이다. 판단의 본체는
`/scan` + 실측 속도다. 맵은 **아는 벽을 버리기 위해**만 본다 (레이싱라인이 벽에
붙어 달려 코너마다 오제동하는 걸 막는다). 맵/TF 가 없으면 필터를 끄고 전부
위험으로 본다 (fail-safe). 플래너·FGM 이 죽거나 헛돌아도 살아 있어야 하므로
일부러 단순하게 유지한다.

```
/scan + /vehicle/speed_mps ──▶ emergency_brake_node ──/emergency_brake──▶ control_node
```

판단 기준은 충돌까지 남은 시간(iTTC)이다.

\[ \text{TTC}_i = r_i / (v \cos\theta_i) \]

**주행 코리도 게이팅이 오작동을 가른다.** 차가 지금 조향각으로 계속 간다고 보고
그 궤적에서 반폭 이내인 빔만 위험으로 센다. 이게 없으면 코너 진입마다 정면 벽
때문에 계속 걸린다. `/drive` 가 끊기면 직선 코리도로 되돌아가는데, 이쪽이 더
보수적(=더 잘 멈춤)이라 fail-safe 다.

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `ttc_threshold_s` | `0.55` | 이 시간 안에 부딪힐 것 같으면 제동. 낮추면 늦게 개입 |
| `arm_speed_mps` | `0.4` | 이 속도 미만이면 **TTC 판정만** 끔 (standoff 는 계속 동작) |
| `min_standoff_m` | `0.30` | 코리도 내 최소 이격. **속도 무관** |
| `standoff_release_m` | `0.40` | 이만큼 멀어져야 해제 |
| `min_hit_beams` | `3` | 노이즈 빔 하나로 급정거하지 않도록 |
| `ego_half_width_m` | `0.17` | 차폭 절반. `corridor_margin_m`(0.05) 를 더해 코리도 |
| `fov_deg` | `100` | 전방 ±50°만 검사 |
| `use_steering_corridor` | `True` | 조향각으로 코리도를 휨 |
| `min_hold_sec` | `0.4` | 한 번 걸리면 최소 유지 (채터링 방지) |
| `release_ttc_factor` | `1.5` | 해제는 트리거보다 느슨하게 |

**판정 조건이 두 개인 이유.** TTC 만 쓰면 구멍이 생긴다. 감속해서 `arm_speed_mps`
아래로 내려가는 순간 TTC 판정이 꺼져 "위험이 사라졌다"고 오인하고, 다시 가속 →
재트리거 → 감속을 반복하며 **조금씩 장애물을 갉아먹고 전진하다 결국 들이받는다**
(시뮬 확인: 4 m 앞 벽에 0.05 m 까지 접근, 제동 27회 토글). 그래서 속도와 무관한
거리 조건 `min_standoff_m` 을 함께 둔다. 이걸 넣으면 0.30 m 에서 멈춰 서서 유지한다.

치워지지 않는 장애물(정지한 상대차 등)은 **여기서 멈춘 채 기다린다.** 돌아가는 건
회피 계층의 일이지 AEB 의 일이 아니다. 장애물이 치워지거나 상대가 움직여야
다시 출발한다.

#### 재발동 방지 — 탈출 창

여유 안쪽에 서 버렸을 때를 위해 `stuck_release_sec`(1.5 s) 뒤 제동을 푸는
탈출구가 있다. 그런데 **푸는 것만으로는 나갈 수 없다.** 차가 아직 그 자리라면
`closest` 가 그대로라 다음 틱(20 ms)에 `standoff` 가 다시 물고, 결과는

```
1.9 s 제동 → 0.02 s 해제 → 재제동 → …        (탈출 불가)
```

제동이 풀린 시간이 한 틱뿐이라 차가 움직일 기회가 없다. 그래서 **stuck 으로
풀린 직후에는 재발동을 일정 구간 막는다.** 그동안 FGM+플래너가 탈출 경로를
찾아 실제로 빠져나가야 한다.

무한정 막으면 그냥 AEB 를 끈 것과 같으므로 네 가지로 창을 닫는다.

| 종료 조건 | 파라미터 | 기본 | 의미 |
| --- | --- | --- | --- |
| 실제로 이동 | `escape_min_travel_m` | `0.35` | 속도 적분. TF 가 끊겨도 동작한다 |
| 시간 초과 | `escape_max_sec` | `3.0` | 못 빠져나갔으면 다시 AEB 에 맡긴다 |
| 정상 주행 복귀 | `escape_speed_end_mps` | `1.0` | 이 속도면 이미 자유 주행이다 |
| **hard_stop 침범** | `escape_hard_stop_m` | `0.12` | **창 안에서도 무조건 제동** |

마지막 줄이 안전의 핵심이다. 이게 없으면 "탈출한다"며 벽으로 그대로 들어간다.
`escape_hard_stop_m` 은 `min_standoff_m`(0.30) 보다 한참 작아야 의미가 있다 —
같게 두면 창이 열리자마자 닫혀서 원래 루프로 돌아간다.

`escape_enable: False` 로 되돌리면 이전 거동(= 재발동 루프)으로 완전히 복귀한다.
로그로 `AEB 탈출 창 시작 #N` / `AEB 탈출 창 종료 — <사유>` 를 찍으므로, 실차에서
`시간 초과` 가 자주 보이면 탈출 경로 자체가 안 나오는 것이니 회피 쪽을 봐야 한다.

**이 창은 절반이다.** 재발동을 막아도 나갈 경로가 없으면 제자리다. 나머지 절반은
플래너 쪽 탈출 모드가 담당한다 → [§5.6.3](#563-aeb-로-멈춘-뒤-빠져나가기--탈출-모드).

> AEB 가 자주 걸리는 것 자체가 이상 신호다. 대부분의 장애물은 `AVOID` 나
> `TRAILING` 이 처리해야 하고, AEB 는 갑자기 튀어나온 것만 받아야 한다.
> `AEB 발동 횟수` 가 랩당 한 자리를 넘으면 회피 진입 임계(`avoid_on_m`)나
> 검출 품질(아래 5.8)을 먼저 의심한다.

> 출발 전 주의: 차를 벽에서 `min_standoff_m`(0.30 m) 안쪽에 놓고 AUTO 로 넘기면
> AEB 가 계속 걸려 출발하지 않는다. 로그에 `EMERGENCY BRAKE [STANDOFF]` 가 뜬다.

제동은 `control_node` 가 한다. AUTO 속도 PI 는 duty 하한이 0 이라 타력주행밖에
못 하므로, AEB 경로에서만 **역토크**(`emergency_brake_duty`, 기본 0.15)를 건다.
거의 멈추면(`emergency_brake_release_speed_mps`) 역토크를 끊는다 — 계속 걸면 후진한다.

**키보드 Space / RC CH6 ESTOP 과 다르다.** 저쪽은 latch 라 사람이 리셋해야 하고,
AEB 는 위험이 사라지면 자동으로 풀린다. AUTO 에서만 동작한다 (MANUAL 은 사람이 판단).

#### 회피 계층과의 간섭 (`/planner/mode`)

회피는 **일부러 장애물 옆을 스치듯** 지난다. AEB 를 평상시 기준 그대로 두면 회피를
시작하는 순간 — 조향이 아직 안 실려서 코리도가 장애물을 정면으로 물고 있을 때 —
제동이 걸려 회피가 죽는다. 그래서 `local_planner` 의 `/planner/mode` 를 구독해
`AVOID`/`REJOIN` 동안만 임계를 낮춘다.

| 상태 | 트리거 | 해제 |
| --- | --- | --- |
| `GLOBAL` (평상시) | ttc < 0.55 s, 거리 < 0.30 m | ttc ≥ 0.83 s, 거리 ≥ 0.40 m |
| `AVOID` / `REJOIN` | ttc < 0.33 s, 거리 < 0.18 m | ttc ≥ 0.49 s, 거리 ≥ 0.28 m |
| `TRAILING` | `GLOBAL` 과 동일 (엄격) | 동일 |
| 토픽 끊김 / 노드 없음 | `GLOBAL` 과 동일 (엄격) | 동일 |

`TRAILING` 은 CSV 를 정상 추종하는 상태라 완화 대상이 아니다 (`avoid_modes` 에
넣지 않는다). 갭 유지에 실패하면 AEB 가 엄격 기준 그대로 받아야 한다.

AEB 탈출 모드([§5.6.3](#563-aeb-로-멈춘-뒤-빠져나가기--탈출-모드))는 `/planner/mode` 를
`AVOID` 로 내보내므로 완화 기준이 걸린다. 의도한 것이다 — 탈출 중에는 장애물
가까이서 조향으로 빠져나가야 하고, 속도는 `aeb_escape_speed_mps`(0.8 m/s) 로 묶여
있다. 그래도 `avoid_standoff_floor_m`(0.18 m) 과 `escape_hard_stop_m`(0.12 m) 은
살아 있어서 밀고 들어가지는 못한다.

**끄는 게 아니라 낮추는 것**이 핵심이다. AEB 는 최후 방어선이라 플래너가 무력화할
수 있으면 의미가 없다. 그래서 두 겹으로 막는다.

- `avoid_*_scale` 을 0 으로 줘도 `avoid_ttc_floor_s`(0.25 s) / `avoid_standoff_floor_m`
  (0.18 m) 아래로는 안 내려간다. 회피 중이어도 0.18 m 면 무조건 선다.
- `/planner/mode` 가 `mode_stale_sec`(0.5 s) 넘게 안 오면 자동으로 엄격 기준 복귀.
  플래너가 죽어서 완화가 걸린 채 방치되는 상황이 없다.

| 파라미터 | 기본 | 설명 |
| --- | --- | --- |
| `avoid_modes` | `[AVOID, REJOIN]` | 완화를 적용할 플래너 상태 |
| `avoid_ttc_scale` / `avoid_ttc_floor_s` | `0.6` / `0.25` | TTC 완화 배율과 하한 |
| `avoid_standoff_scale` / `avoid_standoff_floor_m` | `0.6` / `0.18` | 이격거리 완화 배율과 하한 |
| `mode_stale_sec` | `0.5` | 이 시간 넘게 mode 가 안 오면 엄격 기준 |

회피가 자꾸 AEB 에 막히면 `avoid_ttc_scale` 을 먼저 낮추고, 그래도 막히면
`avoid_ttc_floor_s` 를 손댄다. floor 를 건드는 건 안전 여유를 직접 깎는 것이므로
차폭·제동거리를 다시 계산해 보고 정하는 게 좋다.

신호가 오다가 끊기면 노드가 죽은 것으로 보고 제동한다. 한 번도 못 받았으면
노드를 안 띄운 것으로 보고 무시하므로, AEB 없이 기존처럼 쓸 수도 있다.

```bash
ros2 launch path_following path_follow_stanley_launch.py enable_aeb:=false   # 끄기
ros2 topic echo /emergency_brake/ttc                                          # 튜닝용
```

`/emergency_brake/ttc` 로 현재 최소 TTC 가 나온다(`-1` = 위험 없음). 실차에서
한 바퀴 돌려보고 코너에서 값이 얼마까지 떨어지는지 본 다음 `ttc_threshold_s` 를
그보다 낮게 잡으면 오작동이 없다.

### 5.4 회피 직후 벽 긁힘 방지 (FGM 코리도 검증)

FGM 은 **각도**로 갭을 고른다. 그래서 "각도상으로는 열려 있는데 차폭이 안 들어가는"
통로를 걸러내지 못한다. 특히 `gap_edge_inset_deg`(3°) 는 각도라서 멀수록 실제 여유가
줄어든다 — 3 m 앞에서 3° 는 겨우 **0.16 m** 이고 차량 반폭이 0.17 m 다. 장애물은 잘
피해 놓고 그 옆 벽을 긁는 게 이 때문이다.

그래서 목표점을 찍기 전에 **차폭 코리도로 한 번 더 검증**한다. 목표 방향을 축으로
두고 축에서 `corridor_half_width_m`(0.22 m) 이내로 들어오는 점 중 가장 가까운 것까지가
실제로 갈 수 있는 거리다. 이 거리가 원하는 목표거리보다 짧으면 갭 **안에서** 다른
각도를 훑는다. 점수는

```
min(뚫린거리, 원하는거리) − corridor_straight_bias_m_per_rad × |각도|
```

라서 여유가 충분해지는 순간부터는 정면에 가까운 쪽이 이긴다. 즉 **필요한 만큼만 틀고**
불필요하게 크게 꺾지 않는다. 후보를 원래 갭 범위로 가두므로 검증된 갭 밖으로는 절대
안 나간다. 어느 각도도 안 되면 목표점을 뚫린 데까지 당겨 찍고 경고를 남긴다 —
멈추는 건 AEB 몫이다.

| 파라미터 | 기본 | 설명 |
| --- | --- | --- |
| `corridor_check_enable` | `True` | 끄면 기존(단일 빔) 동작 |
| `corridor_half_width_m` | `0.22` | 차량 반폭 + 여유. 키우면 통로를 더 깐깐하게 봄 |
| `corridor_stop_margin_m` | `0.15` | 막히는 지점에서 이만큼 앞에 목표를 찍음 |
| `corridor_angle_samples` | `11` | 갭 안에서 시도할 각도 후보 수 |
| `corridor_straight_bias_m_per_rad` | `1.0` | 크게 꺾는 데 매기는 벌점. 키우면 정면 고집 |

검증 결과 (합성 스캔):

| 상황 | 결과 |
| --- | --- |
| 개활 | 정면 유지 (0.0°) — 불필요한 조향 안 생김 |
| 왼쪽 0.25 m 에 벽 | −13.5° 로 틀어 벽까지 0.58 m 확보 |
| 폭 0.30 m 통로 (차폭 0.44 m) | 통로에 안 들어가고 뚫린 쪽으로 이탈 |
| 정면 막다른 벽 | 목표를 1.05 m 로 당김 + 경고 |

로그에 이게 뜨면 차폭이 안 들어가는 상황이다 (1초에 한 번만 출력):

```
gap 은 열렸지만 차폭이 안 들어감 — aim=-73° clear=0.99m < 1.00m. 목표점을 당겨 찍음
```

### 5.5 회피 경로 충돌검사

`local_planner` 는 FGM 목표점까지 직선을 긋고, 그 **너머로 `avoid_forward_num_points`
(30) × `avoid_forward_step_m`(0.15) = 4.5 m 를 더 연장**한다. Stanley 가 경로 끝에서
헤매지 않도록 넣은 여유인데, 이 연장 구간은 **아무도 검사한 적이 없다.** 직선이라
코너에서는 그대로 벽을 향한다. 회피는 성공했는데 그 다음에 벽으로 가는 게 이거다.

그래서 완성된 회피 경로를 맵과 장애물로 훑어 **처음 막히는 지점 앞에서 자른다.**

- 맵(`/map`)은 받는 즉시 distance transform 을 한 번 돌려 "가장 가까운 벽까지 거리"
  격자로 만들어 둔다. 이후 조회는 배열 인덱싱이라 40 Hz 에서 부담이 없다.
- 미지 영역(`-1`)도 막힌 것으로 본다. 회피에서 모르는 곳을 낙관하면 안 된다.
- 자른 뒤 `path_check_backoff_m` 만큼 더 물러난다. 끝점이 통과 한계에 딱 붙어 있으면
  Stanley 가 그 점을 겨냥하다 긁는다.
- 남은 길이가 `path_check_min_length_m` 미만이면 **회피를 포기**하고 CSV 를 유지한다.
  짧은 경로를 주면 Stanley 가 끝점에서 이상하게 도는 게 더 위험하다. 이때는 속도
  정책이 이미 감속을 걸어 둔 상태이고, 마지막은 AEB 가 받는다.

| 파라미터 | 기본 | 설명 |
| --- | --- | --- |
| `path_check_enable` | `True` | 끄면 기존(무검사) 동작 |
| `map_topic` | `/map` | latch 토픽이라 `transient_local` 로 구독 |
| `path_check_inflation_m` | `0.25` | 차량 반폭 + 여유. 벽에서 이만큼 떨어져야 통과 |
| `path_check_backoff_m` | `0.20` | 충돌 지점에서 더 물러나는 거리 |
| `path_check_obstacle_margin_m` | `0.10` | 장애물 반경에 더할 여유 |
| `path_check_min_length_m` | `0.6` | 이보다 짧아지면 회피 포기 |

검증 결과 (전방 2.0 m 콘 회피 중, 벽까지 거리를 바꿔가며):

| 전방 벽 | 발행된 회피 경로 길이 |
| --- | --- |
| 없음 | 6.85 m (원래 길이) |
| 6.0 m | 5.65 m |
| 4.0 m | 3.55 m |
| 2.5 m | 2.10 m |

수정 전에는 벽이 어디 있든 **항상 6.85 m** 였다.

> `/map` 이 안 올라와 있으면 벽 검사 없이 장애물 검사만 돈다. 조용히 넘어가면
> 검사하는 줄 착각하므로 한 번 경고를 띄운다.

### 5.6 회피 구간 속도

회피 중에는 CSV 속도가 의미 없다. CSV 속도는 "장애물 없는 레이싱라인을 이 곡률로
돈다" 는 가정으로 뽑은 값인데, 회피는 그 라인을 벗어나 훨씬 급한 조향을 한다.
예전에는 회피 중 무조건 `rejoin_speed_scale`(0.5) 일괄 적용이었다 — 4 m 앞 콘이든
1 m 앞 콘이든 똑같이 절반이라 멀면 과하고 가까우면 모자랐다.

이제 매 주기 **물리로 목표속도를 구해** CSV 대비 배율로 내보낸다. 한계 두 개 중
낮은 쪽을 쓴다.

**1. 조향 한계** — 회피로 트는 각도가 만드는 횡가속도.
FGM 목표점까지를 원호로 보면 `kappa = 2·|lat| / L²`, 여기서 `v = sqrt(a_lat / kappa)`.
급하게 틀수록 느려진다. 레이싱라인 속도 프로파일(2.3절)과 같은 원리다.

**2. 정지 한계** — 회피가 실패해도 부딪히기 전에 설 수 있는 속도.

```
v = v_장애물 + sqrt(2 · a_brake · 남은거리)
```

정지 장애물이면 `v_장애물 = 0` 이라 익숙한 `sqrt(2ad)` 가 되고, **움직이는 장애물이면
그만큼 덜 줄여도 된다.** 앞차가 내 속도와 비슷하면 감속이 거의 없다 — 상대속도로
보기 때문이다. 이 항 하나로 정적/동적이 같이 처리된다.

마지막에 `avoid_safety_factor`(0.7)를 곱한다. 센서 지연, FGM 목표점 흔들림, Stanley
추종 오차처럼 위 계산이 모르는 오차 몫이다.

**접근 구간 선감속.** 위 계산은 `AVOID` 뿐 아니라 `GLOBAL` 에서도 돈다. 회피 모드로
바뀌는 순간에 감속을 시작하면 속도가 계단으로 떨어지는데, 장애물이 검출되는
순간부터 거리에 따라 연속적으로 줄이면 그 계단이 없어진다. `GLOBAL` 에서는 조향
한계를 빼고 거리 기반만 건다 — 아직 안 틀고 있으니 횡가속도 한계는 의미가 없다.

**명령 기울기 제한 (slew).** 그래도 장애물이 검출 범위에 처음 들어오는 순간에는
목표속도가 한 번 뚝 떨어진다. 못 따라가는 명령을 그대로 내면 속도 PI 가 포화되고
적분이 쌓이므로, 감속 명령률을 `avoid_a_brake_mps2` 로 제한한다. 가속 방향은
제한하지 않는다 (위험이 사라지면 바로 회복). 40 Hz 폐루프 시뮬 결과:

| | 명령을 못 따라간 최대 폭 | 장애물 도달 속도 |
| --- | --- | --- |
| slew 없음 | 1.08 m/s | 0.60 m/s |
| slew 있음 | **0.00 m/s** | 0.60 m/s |

안전 결과(도달 속도)는 같고 추종성만 좋아진다.

**옆으로 비켜난 장애물은 제외한다.** 횡거리가 `장애물반경 + 차량반폭 + margin` 을
넘으면 진로 밖이라 속도를 안 건다. 이게 없으면 추월하다 상대차와 나란히 서는
순간 거리가 가까워 브레이크를 밟고, 추월을 영영 못 끝낸다.

| 파라미터 | 기본 | 설명 |
| --- | --- | --- |
| `avoid_speed_enable` | `True` | 끄면 기존 `rejoin_speed_scale` 일괄 적용 |
| `avoid_a_lat_mps2` | `4.0` | 회피 조향에서 허용할 횡가속도 |
| `avoid_a_brake_mps2` | `3.0` | 정지 한계에 쓸 감속도 (AEB 보다 보수적으로) |
| `avoid_safety_factor` | `0.7` | 낮출수록 전체적으로 느리고 안전 |
| `avoid_standoff_m` | `0.35` | 장애물 앞 최소 이격 |
| `avoid_speed_min_mps` | `0.6` | 이 아래로는 안 줄임 (기어가지 않게) |
| `avoid_speed_ref_mps` | `2.0` | CSV 에 속도 열이 없을 때의 기준속도 |

실측값이 나오면 `avoid_a_lat_mps2` / `avoid_a_brake_mps2` 부터 채우면 된다.
`scripts/speed_profile.py` 의 `VEHICLE` 과 같은 성격이지만 **여기가 더 보수적이어야
한다.** 저쪽은 미리 아는 매끈한 라인이고 이쪽은 센서로 급조한 경로다.

검증 결과 (CSV 속도 5.0 m/s 구간):

| 정적 장애물 거리 | 배율 | 실제 속도 |
| --- | --- | --- |
| 3.0 m | 0.53 | 2.67 m/s |
| 2.0 m | 0.41 | 2.05 m/s |
| 1.5 m | 0.33 | 1.65 m/s |
| 1.0 m | 0.22 | 1.12 m/s |

| 앞차 (2.0 m 앞, 내 속도 3.0 m/s) | closing | 배율 | 실제 속도 |
| --- | --- | --- | --- |
| 정지 (0.0 m/s) | 3.0 | 0.41 | 2.05 m/s |
| 1.5 m/s | 1.5 | 0.58 | 2.89 m/s |
| 2.5 m/s | 0.5 | 0.58 | 2.89 m/s |
| 3.0 m/s (동속) | 0.0 | 1.00 | 감속 없음 |

마지막 줄은 **회피 모드로 들어가지도 않는다.** 플래너가 동적 장애를 위협으로 보는
조건이 `closing > 0` 이라, 앞차가 나와 같은 속도로 가면 거리가 안 줄어드니 회피할
이유가 없다. 앞차가 느려지는 순간 `closing > 0` 이 되면서 회피가 켜진다.

### 5.6.1 회피가 끝나면 어떻게 글로벌 패스로 돌아오는가

돌아온다. `rejoin_enable` 이 **기본 `True`** 라, 현재 위치에서 레이스라인까지
Frenet quintic 복귀 경로를 그려 `REJOIN` 모드로 붙는다.

순서는 이렇다.

1. 장애물이 `avoid_fgm_gate_m` 밖으로 나가고 `_avoidance_fully_cleared` 가
   `avoid_off_count_th`(3) 사이클 연속 성립
2. 현재 (s, d, d′) 에서 d → 0 으로 가는 quintic 을 생성.
   길이는 속도 연동 `L = clip(rejoin_time_sec × v_ego, 0.50, 2.50)` m
3. 그 경로도 `_truncate_path_at_collision` 을 통과해야 채택된다.
   막히면 `REJOIN` 을 포기하고 바로 `GLOBAL` — CSV 로 두는 편이 안전하다
4. `|CTE| ≤ rejoin_finish_lateral_m` 이 되면 `REJOIN → GLOBAL`

`rejoin_enable=False` 로 끄면 2번 대신 그냥 `/planner_path_override_active` 를
내려서, Stanley 가 CSV 로 알아서 붙게 둔다 (예전 동작).

> **길이 고정 버그**: 예전에는 `L = min(rejoin_min_length_m, rejoin_max_length_m)`
> 이라 속도와 무관하게 항상 0.50 m 였고 `rejoin_time_sec` 은 읽히기만 하고 안
> 쓰였다. 3 m/s 에서 0.5 m 는 0.17 초 만에 붙으라는 요구라 조향이 튄다.

> **주의 — 과거 동작**: 예전에는 `rejoin_enable` 이 꺼져 있어도 `|CTE| ≤ 0.20 m`
> 가 될 때까지 모드가 `AVOID` 에 남았다. 3번에서 이미 CSV 로 복귀하는 중인데도
> 라벨만 `AVOID` 라서 (a) 회피 속도 상한이 계속 걸리고 (b) AEB 완화가 필요 이상으로
> 오래 유지됐다. 지금은 rejoin 을 쓸 때만 CTE 를 기다린다. 또한 회피 속도 정책은
> 모드 라벨이 아니라 **실제 override 발행 여부**를 보고, `/fgm_target` 이
> `fgm_target_stale_sec` 를 넘겨 오래됐으면 조향 상한을 걸지 않는다.

### 5.6.2 못 지나갈 때 — `TRAILING`

옆으로 빠질 틈이 없으면 예전에는 `AVOID` 를 계속 시도하다 결국 AEB 가 급정거로
받았다. head-to-head 에서는 이게 곧 실격이거나 추돌이다. `TRAILING` 은 그
사이를 메우는 상태다 — **경로는 CSV 그대로 두고 속도만 줄여 갭을 유지한다.**

| 전이 | 조건 |
|---|---|
| `GLOBAL → TRAILING` | 전방 s 갭 ≤ `trailing_enter_m`(3.0 m) 인데 `AVOID` 진입 조건은 안 섬 |
| `AVOID → TRAILING` | 회피 경로가 `path_check_min_length_m` 밑으로 잘렸고 **따라갈 앞차가 있을 때** |
| `AVOID → GLOBAL` | 같은데 따라갈 앞차가 **없을 때** (= 정적 장애물) |
| `TRAILING → AVOID` | 회피 진입 조건이 다시 서고 경로 막힘 래치도 풀림 |
| `TRAILING → GLOBAL` | 갭 > `trailing_exit_m`(4.5 m) 가 `trailing_exit_count_th`(5) 프레임 연속 |

#### 경로 막힘은 시간 래치로 붙든다 — `avoid_retry_sec`

"회피 경로가 막혔다" 는 예전에 한 프레임짜리 bool 이었다. `AVOID → TRAILING`
전이에서 바로 리셋돼 다음 프레임에 `TRAILING → AVOID` 로 튕겨 나갔고, 40 Hz 에서
**2 프레임 주기로 왕복**했다. 그러면 AEB 완화 기준이 프레임마다 깜빡이고
(`AVOID` 프레임은 완화, `TRAILING` 프레임은 엄격) 갭 제어의 미분 이력도 매번
리셋된다. 즉 두 상태 어느 쪽도 제 일을 못 한다.

`avoid_retry_sec`(0.5 s) 동안 "막힘" 을 붙들어 두면 재시도가 그 주기로만 일어난다.
`GLOBAL → AVOID` 도 같은 래치로 막아서, 실패한 회피를 즉시 다시 시도하지 않는다.
대가는 "틈이 생겼는데 최대 0.5 초 늦게 `AVOID` 로 복귀" 뿐이다.

측정 (정적 장애물이 경로를 계속 막는 상황, 2 초 / 80 프레임):

| | 모드 전이 횟수 |
|---|---|
| bool (예전) | 약 78 회 |
| 시간 래치 (0.5 s) | 6 회 |

**따라갈 앞차가 없으면 `TRAILING` 이 아니라 `GLOBAL` 로 보낸다.** 정적 장애물을
`TRAILING` 으로 보내면 전방 s 갭이 `inf` 라서 5 프레임 뒤 곧바로 `GLOBAL` 로
빠져나가고, `AVOID → TRAILING → GLOBAL → AVOID` 가 200 ms 주기로 돈다. 처음부터
`GLOBAL` 로 보내면 AEB 완화 없이 엄격한 기준을 유지한다. 어느 쪽이든 경로는
발행되지 않으므로 Stanley 는 CSV 를 타고, 감속은 모드와 무관한 속도 정책이 한다.

#### 무엇을 "따라갈 앞차" 로 볼 것인가

추월 로직이 없으므로 같은 방향으로 달리는 차는 비켜 가려 하지 말고 뒤에 붙는다.
반대로 아래는 따라갈 대상이 아니라 **`AVOID` 로 보낸다.**

| 앞차 상태 | 판정 | 이유 |
| --- | --- | --- |
| 정지 / `trailing_min_leader_speed_mps`(0.5) 미만 | `AVOID` | 서 있는 차를 따라가면 영원히 그 뒤에 선다 |
| 역주행 (`vs < 0`) | `AVOID` | 마주 오는 차 뒤에 붙는다는 말이 성립하지 않는다 |
| 같은 방향 주행 | `TRAILING` | 뒤에 붙어 갭만 지킨다 |

절대속도만 보면 0.5 m/s 만 넘으면 따라가게 된다 — 우리가 5 m/s 를 낼 수 있어도
1 m/s 로 기어가는 차 뒤에 붙어 같이 1 m/s 로 간다. 레이싱에서 그건 지는 것이라
상대속도 조건을 하나 더 본다.

| 파라미터 | 기본 | 의미 |
| --- | --- | --- |
| `trailing_speed_deficit_enable` | `True` | 아래 조건을 쓸지 |
| `trailing_max_speed_deficit_mps` | `0.5` | `CSV 목표속도 − 앞차속도` 가 이걸 넘으면 `AVOID` |

"비슷한 속도면 따라가고, 확실히 느리면 비켜 간다" 가 된다. 새 상태나 추월 판단
로직은 없다 — **분류 기준만 하나 늘린 것**이고, 비켜 가는 경로는 정적 장애물과
똑같은 FGM 반응형이다.

> 임계를 더 낮추지 말 것. 움직이는 차 옆을 지나는 건 콘을 지나는 것과 다르다.
> 상대는 우리가 옆에 붙은 순간 라인을 바꿀 수 있고 반응형 회피는 그걸 예측하지
> 못한다. 조금만 빨라도 비켜 가려 들면 head-to-head 접촉 위험이 실제로 올라간다.

`TRAILING` 중에는 `/local_path` 를 **발행하지 않고** `override=False` 를 유지한다.
Stanley 는 평소처럼 CSV 를 탄다. 속도만 갭 제어로 조인다.

#### 갭 제어의 기준은 CSV 속도가 아니라 앞차 속도다

처음에는 CSV 속도에 배율을 곱했다. 그런데 그러면 갭이 목표에 맞았을 때
(`gap_error = 0`) 배율이 1.0 이 되어 **CSV 전속**이 나온다. 앞차가 1.2 m/s 인데
자차는 5 m/s 를 명령하니 갭이 순식간에 무너지고, 그때서야 오차가 음수로 커져
급제동한다. 서면 갭이 벌어져 다시 전속. 이 왕복이 "갔다 멈췄다" 버벅임의 정체다.
**정상상태가 없는 제어**였다.

앞차 속도를 기준으로 두면 오차 0 에서 `v = v_lead` 라 정상상태가 생긴다.

```
gap_error = 전방 s 갭 − trailing_target_gap_m
v         = v_lead + Kp·gap_error + Ki·∫ + Kd·d(gap_error)/dt
v         = min(v, v_lead + √(2·a_brake·max(0, gap_error)))   ← 제동거리 상한
v         = min(v_csv, max(0, v))
```

D 항이 중요하다(`Kd=0.25`). 앞차와의 상대속도가 곧 갭의 변화율이라, 거리보다
"좁혀지는 속도"에 먼저 반응해야 붙지 않는다. `Ki` 는 windup 때문에 기본 0.

제동거리 상한이 두 번째 핵심이다. P 항만 두면 갭이 넓을 때 CSV 전속을 명령하고,
목표갭에 닿았을 땐 이미 그 거리 안에서 못 서는 속도가 돼 있다. 그러면 AEB 가
대신 잡는데 AEB 는 역토크라 급정거 → 갭 벌어짐 → 다시 전속으로 버벅인다.
접근 자체를 "목표갭에 맞춰 설 수 있는 속도" 로 제한해야 AEB 를 안 부른다.

> 배율 하한(`trailing_min_speed_scale`)은 제거했다. 절대속도 기준으로 바뀌면서
> 쓰이지 않게 됐고, 하한을 두면 갭이 무너졌을 때 필요한 만큼 못 늦춰서 오히려
> AEB 를 부른다.

갭은 유클리드 거리가 아니라 **s 차이**로 잰다. 코너에서 옆 차선 차가 유클리드로는
가까워도 s 로는 나란히거나 뒤일 수 있다. `_delta_s()` 가 랩 랩어라운드까지 본다.

pose 를 못 얻어 갭을 못 재는 프레임은 "앞차가 사라졌다"로 세지 않는다. 그렇게
세면 TF 가 한 번 끊길 때마다 `TRAILING` 이 풀려 앞차 쪽으로 다시 가속한다.

**AEB 와의 관계.** `TRAILING` 은 AEB 를 대체하지 않는다. `avoid_modes` 는
`[AVOID, REJOIN]` 그대로라 `TRAILING` 중 AEB 는 **엄격 기준**으로 돈다 — CSV 를
정상 추종 중이니 완화할 이유가 없다. `TRAILING` 이 제 몫을 하면 AEB 발동이
줄어야 하고, 그건 로그로 본다.

```
[TRAILING] gap=1.15m target=1.50m v=2.73m/s ego=2.10m/s aeb_total=0
```

`aeb_total` 은 `/emergency_brake` 상승엣지 누적이다. 이 값이 계속 오르면
`trailing_target_gap_m` 을 키우거나 `trailing_kp` 를 올려야 한다.

### 5.6.3 AEB 로 멈춘 뒤 빠져나가기 — 탈출 모드

AEB 쪽에는 이미 재발동을 막는 탈출 창이 있다([§4 탈출 창](#재발동-방지--탈출-창)).
그런데 **재발동을 막는 것만으로는 나갈 수 없다.** 나갈 경로가 있어야 한다.

문제는 정면이 막혀 멈춘 상황에서 플래너가 `TRAILING` 이나 `GLOBAL` 에 있다는
것이었다. 둘 다 `/local_path` 를 발행하지 않으니 Stanley 는 CSV 를 그대로 탄다.
정적 장애물이 CSV 위에 있으면 **CSV 를 따라간다는 건 장애물로 다시 들어간다는
뜻**이다. 결과:

```
AEB 제동 → 해제(탈출 창) → CSV 로 0.6 m/s 기어감 → 다시 접근 → 창 만료 → AEB …
```

조향을 틀 경로가 없어서 제자리에서 장애물을 밀며 왕복한다. AEB 쪽 탈출 창이
무의미해지는 것이다.

그래서 **AEB 로 멈춘 뒤에는 `TRAILING` 대신 `AVOID` 를 강제해 FGM 경로를
발행한다.** `_update_mode` 의 다른 모든 전이보다 먼저 판정한다.

| 파라미터 | 기본 | 의미 |
| --- | --- | --- |
| `aeb_escape_enable` | `True` | 끄면 이전 거동(= 경로 없이 CSV) 으로 복귀 |
| `aeb_escape_arm_speed_mps` | `0.20` | **이 속도 이하로 실제로 멈춘 뒤에만** 경로를 바꾼다 |
| `aeb_escape_hold_sec` | `2.0` | AEB 해제 후 이만큼 더 유지 (빠져나가는 구간) |
| `aeb_escape_speed_mps` | `0.8` | 탈출 중 속도 상한 |
| `aeb_escape_min_path_m` | `0.25` | 탈출 중에만 완화하는 최소 경로 길이 |

두 구간으로 나뉜다.

1. **AEB 가 걸린 채 멈춰 있는 동안** — `control_node` 는 AEB 중에도 조향은
   `/drive` 를 따르므로, 정지 상태에서 바퀴가 탈출 방향으로 미리 꺾인다.
   FGM 목표점 스무딩(EMA)이 수렴할 시간도 여기서 번다.
2. **AEB 해제 후 `aeb_escape_hold_sec`** — 그 방향으로 실제로 빠져나간다.

`aeb_escape_arm_speed_mps` 조건이 안전상 중요하다. 아직 고속으로 제동 중일 때
조향을 새 경로로 틀면 거동이 예측 밖으로 간다. **멈춘 뒤에만** 바꾼다.

두 가지를 탈출 중에만 완화한다. 정면이 막힌 상황에서 평소 기준을 그대로 요구하면
경로가 늘 기각돼 탈출이 성립하지 않기 때문이다.

- 회피 필요 여부 게이트(`_static_wants_fgm_local_path` 등)를 건너뛴다.
- 경로 최소 길이를 `path_check_min_length_m`(0.6 m) → `aeb_escape_min_path_m`(0.25 m).

완화하지 않는 것: `_truncate_path_at_collision` 의 **충돌 검사 자체**는 그대로다.
벽·장애물을 관통하는 경로는 탈출 중에도 발행되지 않는다. 그래서 정말 나갈 틈이
없으면 경로가 안 나오고 차는 그 자리에 선다 — 장애물을 밀고 가는 것보다 낫다.

속도는 `aeb_escape_speed_mps` 로 덮어쓴다(우선순위는 AEB 다음, 나머지보다 위).
이게 없으면 장애물이 시야에서 빠지는 순간 CSV 전속으로 튀어 나간다.

로그로 진입/종료를 한 번씩 찍는다.

```
AEB 탈출 모드 진입 — FGM 회피경로 강제 발행, 속도 ≤0.80m/s (aeb_total=3)
AEB 탈출 모드 종료 — 정상 판정 복귀
```

### 5.6.4 회피 경로 모양 — `avoid_path_mode`

| 값 | 방식 |
|---|---|
| `straight` (기본) | FGM 목표점까지 직선 + `avoid_forward_step_m × avoid_forward_num_points` 전방 직선 연장 |
| `frenet` | 레이스라인을 기준선으로 `d(s)` quintic — 진입 → 유지(apex) → 복귀 |

`frenet` 은 기준선의 곡률을 그대로 따라가므로 고속 코너에서 조향이 덜 급하고
복귀가 매끄럽다. 구간 길이는 `avoid_frenet_enter_len_m` / `_hold_m` / `_exit_len_m`
이고, `|d|` 는 `avoid_frenet_max_offset_m`(0.65 m) 로 클램프된다. 클램프된 경로가
그래도 막히면 `_truncate_path_at_collision` 이 잘라내고, 그 결과 막힘 래치가 서서
`TRAILING`(앞차 있음) 또는 `GLOBAL`(정적) 로 넘어간다.

생성에 실패하면 조용히 `straight` 로 대체하고 경고를 남긴다. 검증 전까지 기본값은
`straight` 다.

### 5.6.5 Frenet 스냅샷 (`/planner/frenet_debug`)

매 주기 자차와 필터 통과 장애물을 CSV 폐곡선에 투영해 `(s, d)` 로 들고 있는다.
`TRAILING` 갭과 예측 s 가 이걸 쓴다. **기존 XY 기반 게이트(`avoid_on_m` 등)는
그대로다** — 정보만 추가한 것이지 판정 로직을 바꾼 게 아니다.

`publish_frenet_debug:=true` 로 켜면 `[s_ego, d_ego, s_obs1, d_obs1, ...]` 가
`Float32MultiArray` 로 나온다. pose 가 없으면 앞 두 값이 `NaN`.

`use_predicted_s:=true` 면 동적 장애물의 s 를 `pred_horizon_sec`(1.0 s) 만큼
등속 전파한 값을 회피 진입 판정과 갭 계산에 쓴다. 앞차가 빠르게 멀어지는 중이면
불필요한 감속을 줄여 준다. 전방/후방 판정은 **항상 현재 s** 로 한다 — 예측 s 로
거르면 마주 오는 물체가 "이미 지나갔다"고 계산되어 갭 계산에서 사라진다.

동적 장애물의 `vx, vy` 는 laser frame **상대속도**다. 절대 s 속도는
`R(yaw)·v_laser + v_ego` 로 근사한다 (자차 요레이트 항은 1 초 예측이라 무시).

### 5.7 고속(7 m/s급)에서 회피가 되는가

직선 최고속을 올리면 회피 쪽에서 먼저 한계가 온다. 어디서 막히는지 정리해 둔다.

**1. 조향 한계 — 목표점 거리가 정한다.** 같은 횡오프셋이라도 목표점이 가까우면
곡률이 커져 횡가속도 한계에 먼저 걸린다. 검출 거리를 아무리 늘려도 이 천장은
안 올라간다.

| FGM `target_max_m` | 횡 0.5 m 회피 시 천장 |
| --- | --- |
| 3.5 m (예전 값) | 4.90 m/s |
| 4.0 m | 5.60 m/s |
| **5.0 m (현재)** | **7.00 m/s** |

그래서 `target_max_m` 을 5.0 으로, 그걸 담으려고 `scan_max_range_m` 을 5.5 → 10.0
으로 올렸다. 저속에서는 목표점이 `속도 × 0.7` 이라 영향이 없다 (2 m/s 면 1.4 m).

> `scan_max_range_m` 은 갭 탐색 자체에 영향을 준다. 예전엔 5.5 m 밖이 전부
> "뚫림" 으로 뭉개졌는데 이제 10 m 까지 실제 거리로 보인다. 좁은 실내 트랙에서는
> 갭 선택이 달라질 수 있으니 실차에서 한 번 확인하는 게 좋다.

**2. 정지 한계 — 검출 거리와 `a_brake` 가 정한다.** 이쪽은 `sqrt(2·a·d)` 라
검출 거리에 물린다. `avoid_on_max_m` 을 7.5 → 12.0 (검출 상한과 동일) 으로 올렸다.

| 검출/진입 거리 | `a_brake`=3.0 | `a_brake`=4.5 |
| --- | --- | --- |
| 7.5 m (예전) | 4.58 m/s | 5.61 m/s |
| **12.0 m (현재)** | **5.85 m/s** | 7.16 m/s |

7 m/s 를 계단 없이 받으려면 `a_brake` 실측이 4.3 이상 나와야 한다. 그 아래면
검출 순간(12 m)에 7.00 → 5.85 로 한 번 떨어지는데, **이건 slew 가 3 m/s² 로
완만하게 깔아준다.** 못 따라가는 명령은 아니다.

수정 후 7 m/s 진입 프로파일 (정면 콘, CSV 7.0 m/s):

| 표면 거리 | 모드 | 목표 속도 | 배율 |
| --- | --- | --- | --- |
| 14 m | GLOBAL | 7.00 | 1.00 |
| 12 m | AVOID | 7.00 → 이후 3 m/s² 로 하강 | 1.00 |
| 9 m | AVOID | 6.97 | 1.00 |
| 7 m | AVOID | 6.08 | 0.87 |
| 5 m | AVOID | 5.05 | 0.72 |
| 3 m | AVOID | 3.74 | 0.53 |
| 2 m | AVOID | 2.87 | 0.41 |

수정 전에는 7.5 m 에서 7.00 → 4.58 로 **한 번에 떨어졌고**, 최대 제동으로도
허용선을 끝까지 못 따라잡아 장애물에 2.0 m/s 로 도달했다.

**더 올리려면** `avoid_a_lat_mps2`(조향)와 `avoid_a_brake_mps2`(제동) 실측이
필요하다. 지금 4.0 / 3.0 은 보수적으로 잡은 값이라, 실제 그립이 더 나오면
그만큼 천장이 올라간다. 검출 거리(12 m)를 더 늘리는 건 라이다·장애물 인식
품질 문제라 그 다음 순서다.

### 5.8 검출 품질 (클러스터링 / 트래킹)

회피 판단의 입력이 전부 여기서 나온다. 경로가 아무리 좋아도 장애물이 안 보이거나
반지름이 프레임마다 튀면 소용이 없다. 아래 다섯 개는 **전부 기본값이 기존 동작**
이고, 플래그 하나만 되돌리면 원복된다.

클러스터링은 `path_following/scan_cluster.py` 로 합쳤다. 예전에는
`static_obstacle_node._cluster_xy` 와 `integrated_obstacle_node._cluster_indices` 에
같은 알고리즘이 따로 있어서, 한쪽만 고치면 두 런치의 거동이 조용히 갈라졌다.

| 플래그 | 기본 | 켜면 | 노드 |
| --- | --- | --- | --- |
| `cluster_mode` | `fixed` | `adaptive` — 끊는 임계를 거리에 비례 | static / integrated |
| `adaptive_min_points` | `False` | 원거리에서 최소 점수를 낮춤 | static / integrated |
| `consistent_centroid` | `False` | 추적은 centroid, 반지름은 분위수 | static / integrated |
| `tracker_mode` | `ema` | `kf` — 등속 칼만 | integrated |
| `wall_residual_guard` | `False` | 팽창 벽에 붙은 잔차에 높은 기준 | static / integrated |
| `bubble_speed_scale_enable` | `False` | FGM 버블을 속도에 비례 | fgm |

**적응형 임계(`cluster_mode: adaptive`).** 끊는 기준을
`d_max(r) = r·sin(Δφ)/sin(λ−Δφ) + 3σ` 로 잡고 `[0.05, 0.35] m` 로 클램프한다.
`λ`(기본 10°)는 "이보다 비스듬한 면은 같은 물체로 안 본다"는 허용 입사각이다.

여기서 알아둘 것: 고정 `0.28 m` 는 사실상 **8 m 용으로 튜닝된 값**이다. λ=10°
에서 두 곡선은 8 m 부근에서 만난다. 즉 적응형으로 바꾸면 원거리가 아니라
**근거리가 크게 엄격해진다** (2 m 에서 0.115 m). 가까운 물체를 잘 분리하는 게
이득이지만, 반대로 **가까운 상대차 하나가 2~3 조각으로 쪼개질 위험**이 있다.
조각 각각이 `min_obstacle_size_m`(0.14) 미만이면 전부 버려져서 **장애물이 통째로
사라진다.** 실차에서 이 모드를 켤 때는 상대차를 2 m 앞에 세워두고
`/static_obstacles` 개수가 1 로 유지되는지부터 봐야 한다. 쪼개지면
`abd_lambda_deg` 를 15~20° 로 올린다 (임계가 커진다).

**거리 스케일 최소 점수(`adaptive_min_points`).** 10 m 앞 0.3 m 물체는
각분해능상 7점밖에 안 찍혀서 고정 10점 기준에 걸려 통째로 사라진다.
`min_arc_m / (r·Δφ)` 로 기대 점수를 계산해 `min_cluster_points_floor`(3)까지
낮춘다. **`wall_residual_guard` 와 같이 켜는 걸 권한다** — 안 그러면 노이즈
3점이 장애물이 된다.

**대표점 일관화(`consistent_centroid`).** 지금은 `laser_x/y` = 최근접점,
`map_x/y` = 평균이라 정의가 다르다. 그 불일치가 유한차분 속도에 그대로 노이즈로
들어간다. 켜면 **추적·속도는 centroid, 발행·거리 게이트는 최근접점**으로 갈라
쓴다. 토픽 레이아웃(`[id,x,y,r]` / `[id,x,y,vx,vy,r]`)과 프레임은 그대로라
소비자(AEB·FGM·플래너)는 아무것도 모른다. 반지름도 bbox `span/2` 대신 중심에서의
점 거리 `radius_percentile`(90) 분위수를 써서 이상점 몇 개에 덜 끌린다.

**등속 칼만 트래커(`tracker_mode: kf`).** `x = [px, py, vx, vy]`, 관측은 위치만.
map / laser 프레임에 각각 하나씩 돌린다 — laser 쪽이 상대운동이라 `closing_mps`
가 거기서 바로 나온다 (부호 규약 `-(p·v)/|p|` 는 그대로). 미검출 프레임에도
`predict` 를 돌려 트랙이 얼어붙지 않게 한다.

단위 테스트에서 확인된 차이 (`test_obstacle_tracking.py`):

- 3 프레임(0.075 s) 가림 후 재검출에서 **EMA 속도는 참값의 1.5배 이상 튄다.**
  얼어 있던 위치 때문에 4 프레임치 변위가 한 `dt` 에 몰려서다. KF 는 1.2배 미만.
- **`track_keep_time_s`(0.12) 는 40 Hz 에서 5 프레임뿐이다.** 그보다 긴 가림이면
  트랙이 삭제되고 새 ID 로 태어나면서 `age_s` 가 리셋되고, `dynamic_confirm_time_s`
  를 다시 세는 동안 **달려오는 차가 static 으로 분류된다.**

유지 시간은 `tracker_mode` 에 묶여 있다. `ema` 는 `track_keep_time_s`(0.12),
`kf` 는 `track_keep_time_s_kf`(0.25) 를 쓴다. `ema` 에서 늘리면 **얼어붙은 위치가
그대로 오래 남아 오히려 나빠지므로** 같이 올리면 안 된다. `kf` 는 미검출 중에도
`predict` 로 위치를 밀어 주니 늘려도 말이 된다. 묶어 둔 덕에 `tracker_mode` 하나만
되돌리면 유지 시간도 같이 원복된다. 노드 시작 로그에 `tracker=kf keep=0.25s` 로
실제 적용값이 찍힌다.

**벽 잔차 가드(`wall_residual_guard`).** 측위가 흔들리면 벽이 `is_wall()` 팽창
밴드를 벗어나 "장애물"로 샌다. 켜면 팽창 경계에서 `wall_clearance_m`(0.12) 안쪽에
있는 클러스터에는 `near_wall_min_points`(14) + `near_wall_min_span_m`(0.20) 를
추가로 요구한다. 경계 거리는 `StaticMap` 에 distance transform 격자를 **첫 호출에
한 번만** 만들어 조회한다 (끄면 비용 0).

**FGM 버블 속도 연동(`bubble_speed_scale_enable`).** 고정 0.2 m 는 고속에서 너무
작고 저속에서 과하다. `clip(0.18 + 0.035·v, 0.18, 0.40)`. `_select_gap()` 의
히스테리시스와 corridor_check 는 건드리지 않았다.

검증은 ROS 없이 돈다:

```bash
python3 -m pytest src/path_following/test/ -q     # 27 passed
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
| 모드 | CH5 수동/자율, planner GLOBAL/AVOID/REJOIN/TRAILING, Stanley CSV/LOCAL |
| 속도 | `/drive` 명령, `/odom` 또는 TF 추정, VESC duty |
| 조향 | `/drive` rad/deg, ESP `S:` 전송값 |
| LiDAR | `/scan` Hz, 전방 최소거리, `/static_obstacles` 개수·최근접 |
| FGM | `/fgm_target` 거리·방향 |

**필요 토픽**

- `/vehicle/telemetry` ← `control_node` (duty, RC, AUTO/MANUAL)
- `/planner/mode` ← `local_planner_node` (GLOBAL/AVOID/REJOIN/TRAILING)
- `/drive`, `/scan`, `/static_obstacles`, `/fgm_target`, `/planner_path_override_active`

코드 수정 후: `colcon build --packages-select path_following`

---

## 10. 튜닝 요약 (현재 CFG 기본값)

| 노드 | 주요 파라미터 |
|------|----------------|
| `stanley_waypoint_follow_node` | `max_steering_angle` ±40°, `stanley_k` 1.5, `max_drive_speed` 1.5 |
| `local_planner_node` | `avoid_on_m` 1.8, `avoid_pass_rear_x_m` -0.35, 직진 leg 30×0.15 m, `path_check_inflation_m` 0.25, `avoid_a_lat_mps2` 4.0 |
| `drive_strategy_node` | 직선 `speed_straight_mul` 2.0, 곡선 `speed_curve_mul` 0.5 |
| `fgm_node` | 목표점 = `target_lead_time_s` 0.70 × 속도 (1.0~5.0 m), `scan_max_range_m` 10.0, `bubble_radius_m` 0.20, `gap_edge_inset_deg` 3, `corridor_half_width_m` 0.22 |
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
ros2 launch path_following path_follow_static_dynamic_avoid_launch.py
ros2 launch path_following path_follow_stanley_launch.py




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


ws://192.168.137.181:8765

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
