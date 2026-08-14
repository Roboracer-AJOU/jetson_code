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
| 토픽 끊김 / 노드 없음 | `GLOBAL` 과 동일 (엄격) | 동일 |

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

돌아온다. 다만 **복귀 경로를 따로 그려서 돌아오는 게 아니라, 회피 경로 발행을
멈추면 Stanley 가 알아서 CSV 로 붙는 방식**이다. `rejoin_enable` 이 기본 `False`
라서 Frenet quintic 복귀 경로는 만들지 않는다.

순서는 이렇다.

1. 장애물이 `avoid_fgm_gate_m` 밖으로 나가고 `_avoidance_fully_cleared` 가
   `avoid_off_count_th`(3) 사이클 연속 성립
2. 플래너가 `/planner_path_override_active` 를 `False` 로 내림
3. Stanley 가 `/local_path` 를 무시하고 CSV 를 다시 따라감 → 횡오차가 줄어듦
4. 모드가 `AVOID → GLOBAL`, 속도 배율도 1.0 으로 복귀

`rejoin_enable=True` 로 켜면 2번 대신 현재 위치에서 CSV 까지 quintic 을 그려
`REJOIN` 모드로 부드럽게 붙는다. 이때는 `|CTE| ≤ rejoin_finish_lateral_m` 이
될 때까지 모드를 붙들고 있는다.

> **주의 — 과거 동작**: 예전에는 `rejoin_enable` 이 꺼져 있어도 `|CTE| ≤ 0.20 m`
> 가 될 때까지 모드가 `AVOID` 에 남았다. 3번에서 이미 CSV 로 복귀하는 중인데도
> 라벨만 `AVOID` 라서 (a) 회피 속도 상한이 계속 걸리고 (b) AEB 완화가 필요 이상으로
> 오래 유지됐다. 지금은 rejoin 을 쓸 때만 CTE 를 기다린다. 또한 회피 속도 정책은
> 모드 라벨이 아니라 **실제 override 발행 여부**를 보고, `/fgm_target` 이
> `fgm_target_stale_sec` 를 넘겨 오래됐으면 조향 상한을 걸지 않는다.

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
