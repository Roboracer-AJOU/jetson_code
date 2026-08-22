# path_following — Stanley 경로 추종 + 횡오프셋 회피 (Jetson 실차)

F1TENTH Ajou 젯슨 실차용 주행 스택. Cartographer **map** 프레임 CSV를 따라가며, 정적 장애물 회피를 수행한다.

회피는 **계획된 횡오프셋 기동**이 담당한다 (`offset_maneuver.py`, [5.6.4](#564-회피-경로-모양--avoid_path_mode)).
멀리서부터 조금씩 비켜 (6 m/s 에 1.6°) 지나가고 완만하게 라인으로 돌아오며, 제때
봤으면 **감속하지 않는다**. FGM 은 이 계획이 실패할 때의 폴백으로 내려갔다.

**워크스페이스:** `/home/nvidia/f1tenth_ajou`

---

## 노드 구성

```
/static_obstacles  ← static_obstacle_node  (/scan)
/fgm_target        ← fgm_node              (/scan)
/fgm_gap_marker    ← fgm_node              (Marker, Foxglove 기존 레이아웃)
/fgm_gap_markers   ← fgm_node              (MarkerArray, 장애물과 같은 타입)
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

#### 전진으로 못 나갈 때 — 후진 탈출

조향만으로는 못 푸는 자세가 있다. FGM 이 막힌 것으로 보는 반각은

```
asin((장애물반경 + 버블 + 차반폭) / 거리)
```

라서, 거리가 그 분자(약 0.55 m) 안이면 **90°** 다. 최대 조향을 줘도 열린 방향이
없다. 실차에서 이때 탈출 창 3 초를 제자리에서 다 쓰고, 창이 닫히면 standoff 가
다시 물어 영영 그 자리였다. 물러나면 이 식이 풀린다 — 0.4 m 면 50° 로 떨어져
그때부터 전진 탈출이 된다.

그래서 탈출 창이 **아무 진전 없이**(이동 < `escape_min_travel_m`의 절반) 시간
초과로 닫히면 최대 조향 실패로 보고 곧게 물러난다.

**그런데 실차에서는 그 시간 초과까지 가지도 못했다.** `closest` 가
`escape_hard_stop_m` 과 같은 값에 걸터앉으면 (둘 다 0.24) 창이 열린 다음 틱에
hard_stop 침범으로 닫힌다.

```
AEB 탈출 창 시작 #105 → 0.02 s 뒤 EMERGENCY BRAKE [STANDOFF] → 반복
```

실제로 **#105 까지** 돌았다. 제동이 풀린 시간이 한 틱뿐이라 차는 조향만 파르르
떨며 그 자리에 서 있었고, 시간 초과 분기는 영영 오지 않았다. 창의 종료 사유에
후진을 매단 게 잘못이었다.

그래서 창과 무관하게 **결과만 본다.** 서 있는데 앞이 막혔으면 못 나가는
것이다 — 창이 어떤 사유로 닫히든, AEB 가 켜져 있든 꺼져 있든 상관없다.

기다리는 시간은 **얼마나 가까운지로 갈린다.** 코앞이면 1 초를 더 지켜봐야
달라질 게 없다 — 위 식이 이미 90° 라 조향으로 나갈 방법이 없는 거리다. 그래서
바로 물러난다. 반대로 범퍼 0.4 m 쯤 떨어져 있으면 아직 조향으로 빠져나갈
여지가 있으므로 예전대로 1 초를 지켜본다.

| 파라미터 | 기본 | 의미 |
| --- | --- | --- |
| `reverse_close_obstacle_m` | `0.44` | 이 안이면 **코앞** — 오래 안 본다 (범퍼 0.25 m) |
| `reverse_close_stuck_sec` | `0.3` | 코앞일 때 기다리는 시간 |
| `reverse_stuck_sec` | `1.0` | 코앞이 아닐 때 이만큼 서 있으면 전진 실패로 본다 |
| `reverse_stuck_obstacle_m` | `0.79` | 이 안에 뭔가 있어야 "막혀서" 선 것 (범퍼 0.6 m) |
| `reverse_cooldown_sec` | `1.5` | 물러난 뒤 이만큼은 다시 안 건다 |

`reverse_close_stuck_sec` 이 0 은 아니다. VESC 속도가 순간 0 을 찍는 것과 진짜
정지를 가르려면 몇 틱은 봐야 한다.

`reverse_stuck_obstacle_m` 이 정차와 갇힘을 가른다. 앞이 비어 있는데 서 있는
것은 출발 대기지 갇힌 게 아니므로 후진하지 않는다. 쿨다운이 없으면 물러난
직후 또 걸려서 뒤가 빌 때까지 뒷걸음질한다.

MANUAL 에서는 집행하는 `control_node` 가 AUTO 게이트로 막으므로, 요청이 나가도
아무 일도 일어나지 않는다.

| 파라미터 | 기본 | 의미 |
| --- | --- | --- |
| `reverse_escape_enable` | `True` | `False` 면 이전 거동(그대로 정지) |
| `reverse_travel_m` | `0.20` | 한 걸음. 이만큼 물러나면 끝내고 전진 재시도 |
| `reverse_max_sec` | `2.0` | 속도계가 0 이라 거리가 안 쌓여도 끊는다 |
| `reverse_min_clearance_m` | `0.45` | 뒤가 이만큼 안 비면 **시작하지 않는다** |
| `reverse_abort_clearance_m` | `0.25` | 후진 중 여기까지 좁아지면 즉시 중단 |
| `reverse_map_margin_m` | `0.10` | 맵 검사에서 차 반폭에 더할 여유 |

한 걸음이 0.20 m 인 건 **끊어 가려는** 것이다. 예전 0.40 은 전진 탈출이 열리는
기하학적 최소치라 한 번에 끝내려던 값인데, 그러려면 뒤 여유를 0.60 m 요구하게
되고 트랙 폭을 생각하면 그 조건이 자주 안 맞아 `전진도 후진도 막혔다` 로
빠졌다. 지금은 0.20 씩 가고, 한 걸음으로 안 열리면 쿨다운 뒤 또 한 걸음 간다 —
매번 뒤 여유를 다시 확인하는 셈이라 뒤가 좁은 자리에서도 할 수 있는 만큼은 한다.
`reverse_min_clearance_m` 0.45 는 한 걸음(0.20)에 중단 임계(0.25)를 더한 값이다.

#### 뒤는 맵으로 본다 (라이다가 뒤를 못 본다)

처음엔 스캔의 후방 섹터를 쟀다. **틀렸다 — 이 라이다는 뒤를 못 본다.** 후방에
빔이 아예 없으니 대부분 `inf`(비었다)가 나오고, FOV 가장자리 잡음이 하나
들어오면 `0.00` 이 나온다. 실차에서 이렇게 됐다.

```
후진 탈출 시작 #23 — 뒤 여유 inf m, 0.40m 물러난다
후진 탈출 종료 — 뒤가 막혔다 (0.00m)      ← 0.02 s 뒤
```

20 ms 만에 끝나니 차는 찔끔 움직이고 만다. 조향을 낼 거리도 안 나온다.

그래서 맵과 TF 로 본다. 지금 자세에서 차체 뒤로 0.05 m 씩 훑으며
`clearance_at` 이 `HALF_WIDTH_M + reverse_map_margin_m`(0.25) 미만이 되는
지점까지의 거리를 낸다. 라이다가 축보다 0.31 m 앞이라 뒤끝까지가
`LASER_TO_REAR_M` = 0.41 m 이고, 검사는 거기서부터 시작한다.

중심선 기준이라 실제보다 반폭만큼 일찍 막힌 것으로 본다 — 보수적인 쪽이라
그대로 둔다.

**못 재면 0 이다.** 전방 검사와 정반대다. 앞은 안 보여도 라이다가 어떻게든
보지만 뒤는 볼 수단이 맵뿐이라, 낙관하면 눈 감고 후진하는 셈이 된다. 맵이나
TF 가 없으면 아예 물러나지 않는다. 그래서 `reverse_escape_enable` 이 켜져
있으면 `map_filter_enable` 과 무관하게 맵을 구독한다.

> 한계: **맵에 없는 물건은 뒤에 있어도 모른다.** 사람이나 새로 놓인 상자가
> 뒤에 있으면 그대로 민다. 그래서 한 걸음을 0.2 m 로 짧게, 속도도 기는
> 수준(`escape_reverse_max_speed_mps` 0.6)으로 묶어 둔다.

물러나는 동안은 전방 제동을 끊는다. 앞이 가까운 건 이미 아는 사실이고 그래서
뒤로 가는 중이다. 다 물러나면 전진 탈출 창을 새로 열어 준다.

집행은 `control_node` 가 `/aeb/escape_reverse` 를 받아서 한다 (역토크와 같은
하드웨어 경로). 다른 점이 둘 있다.

- **조향을 중립으로 되돌린다.** 제동은 마지막 조향을 유지하지만, 꺾인 채
  물러나면 뒤가 어디로 갈지 예측이 안 되는데 뒤 여유는 곧게 간다고 보고 쟀다.
- **신호가 끊기면 푼다.** 제동은 끊기면 거는 게 안전하지만(fail-safe), 후진은
  차를 움직이는 명령이라 반대다. `escape_reverse_stale_sec`(0.3) 지나면 끊고,
  요청이 붙박이 True 가 돼도 `escape_reverse_max_sec`(2.5) 에서 끊는다 — 요청이
  한 번 False 로 떨어져야 다시 걸린다. `escape_reverse_max_speed_mps`(0.6) 를
  넘으면 duty 를 끊고 타력으로 둔다.

앞뒤가 다 막혔으면 `전진도 후진도 막혔다 — 뒤 여유 …` 를 찍고 그대로 선다.
미는 것보다 서 있는 게 낫다.

> AEB 가 자주 걸리는 것 자체가 이상 신호다. 대부분의 장애물은 `AVOID` 나
> `TRAILING` 이 처리해야 하고, AEB 는 갑자기 튀어나온 것만 받아야 한다.
> `AEB 발동 횟수` 가 랩당 한 자리를 넘으면 회피 진입 임계(`avoid_on_m`)나
> 검출 품질(아래 5.8)을 먼저 의심한다.

> 출발 전 주의: 차를 벽에서 `min_standoff_m`(0.30 m) 안쪽에 놓고 AUTO 로 넘기면
> AEB 가 계속 걸려 출발하지 않는다. 로그에 `EMERGENCY BRAKE [STANDOFF]` 가 뜬다.

제동은 `control_node` 가 한다. AUTO 속도 PI 는 duty 하한이 0 이라 타력주행밖에
못 하므로, AEB 경로에서만 **역토크**(`emergency_brake_duty`, 기본 0.15)를 건다.
거의 멈추면(`emergency_brake_release_speed_mps`) 역토크를 끊는다 — 계속 걸면 후진한다.

해제 판정은 **부호 있는** 속도로 한다. 예전에는 `abs()` 였는데, 그러면 이미 뒤로
구르는 중에 |속도| 가 다시 임계를 넘어 역토크가 되살아나고 뒤로 갈수록 더 세게
미는 폭주가 된다. 20 Hz 에서 감속이 해제 구간(±0.15)을 한 틱에 건너뛰면 바로 그
상태가 됐다 — 실차에서 "가끔 뒤로 간다" 던 게 이거다. 의도한 후진은 아래
`/aeb/escape_reverse` 만 낸다.

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

### 5.3.9 장애물이 회피까지 도달하는 경로 — 게이트 두 개

작은 장애물(0.3~0.5 m)이 인식되지 않으면 대부분 이 둘 중 하나다.

**(1) 맵 잔차의 벽 팽창 — `wall_match_radius_m`**

검출 노드는 맵 벽을 이 반경만큼 부풀린 뒤 그 안의 스캔점을 **전부** 지운다.
TF·맵 어긋남을 덮으려면 필요하지만, 키운 만큼 벽 옆 실장애물도 통째로
사라진다. 실제 맵(`20260818_204522`)으로 재 보면 사각지대가 이만큼이다.

| 반경 | 팽창 셀 | 사각지대 | 벽 근처 자유공간 대비 |
|------|---------|----------|----------------------|
| 0.42 | 9 | 82.9 m² | **36.9 %** |
| 0.30 / 0.28 | 6 | 54.9 m² | 24.4 % |
| **0.25** | **5** | **46.1 m²** | **20.5 %** |
| 0.20 | 4 | 35.1 m² | 15.7 % |

`0.42` 는 트랙 가장자리 공간의 **37 %** 를 못 보는 상태였다. `0.25` 로
줄여서 **36.8 m² (16.4 %p)** 를 되찾았다. 팽창은 `ceil(r/resolution)` 셀이라
0.05 배수로만 의미가 있다 — 0.28 과 0.30 은 똑같이 6셀이다.

마지노선의 근거는 7 m/s 기준 오차 예산이다.

| 항목 | 크기 | 비고 |
|------|------|------|
| 측위 잔차 | 0.10 | map→odom 점프 실측 0.10~0.13 |
| 스캔 왜곡 | 0.09 | 25 ms 스윕을 강체변환. 디스큐 없이는 못 없앰 |
| 맵 격자 반칸 + 거리 노이즈 | 0.045 | |
| **합계** | **0.235** | → 격자 올림 **0.25** |

여기서 더 낮추려면 점별 디스큐나 측위 개선이 먼저다. 감으로 내리지 말고
`scripts/measure_wall_residual.py` 로 실측할 것 — 빈 트랙을 한 바퀴 돌면
모든 스캔점이 정의상 벽이므로, 그 점들이 맵 벽 셀에서 떨어진 거리의
p99.5 가 곧 필요한 반경이다.

**그런데 0.25 로도 벽에 붙은 장애물은 통째로 사라진다 (20260822 실측).**

위 표의 "사각지대" 는 면적이라 체감이 안 됐다. 벽에 붙인 50 cm 박스를
3.4 m 앞에 두고 989 스캔을 재 보니, 박스 점들이 맵 벽에서 **0.10~0.43 m**
에 걸쳐 있었다. LiDAR 가 정면과 측면을 같이 보는데 측면은 벽 쪽으로
파고들기 때문이다. 팽창 0.25 가 그 아래를 먹으면 남는 건 10점 / 폭 0.12 m
뿐이고, 이게 `min_obstacle_size_m`(0.12) 에 정확히 걸려 탈락한다.

같은 박스를 벽에 붙였을 때 **31초 내내 검출 0**, 트랙 가운데로 옮기니
**135초 내내 검출 1** 이었다. 회피가 한 박자 늦은 게 아니라 아예 시작을
못 했고, AEB 가 원시 스캔으로 2.5 m 에서 잡아 세우는 게 유일한 반응이었다
(`debug/wall_filter_probe.py`).

`debug/inflation_sweep.py` 는 같은 스캔에 노드와 **동일한 클러스터링·가드**
를 걸고 팽창만 바꿔 본다.

| 팽창 | 벽에 붙은 박스 검출률 | 유령/스캔 | 검출된 폭 |
|------|----------------------|-----------|-----------|
| 0.25 | 0.3 % | 0.00 | 0.26 m |
| 0.20 | 19 % | 0.00 | 0.28 m |
| 0.15 | 83 % | 0.00 | 0.31 m |
| **0.10** | **100 %** | **0.00** | **0.36 m** |

정지 측정이라 위 예산의 스캔왜곡 항(0.09)이 빠져 있다. 그걸 뺀 정지 기준이
0.145 이고 **0.10** 은 거기서 한 칸 더 들어간 값이다 — 남는 위험은 주행 중
유령이고, 그건 아래 가드로 갚는다.

새로 드러나는 띠는 near-wall 가드가 맡는다. 팽창 0.25 + 여유 0.20 이 맵 벽
기준 **0.45 m** 까지를 덮었으므로, 팽창을 0.10 으로 내린 지금은
`wall_clearance_m` 이 **0.35** 여야 같은 범위가 유지된다. 지워지지 않게 된
구간이 아무 판단도 안 받고 통과하는 일은 없어야 한다.

그 띠 안의 클러스터는 `near_wall_min_points` 와 `near_wall_min_span_m`(0.20)
를 넘어야 살아남는다. 다만 고정값 14 는 물리적 크기가 아니라 각분해능에
묶인 수다 — 3.4 m 에서 14점은 호 길이 0.25 m 지만 8 m 에서는 0.58 m 라,
같은 박스가 멀다는 이유만으로 탈락한다. 가드가 덮는 띠를 넓힌 만큼 이
왜곡도 커지므로, `near_wall_point_gate()` 가 그 거리에서 `min_span_m` 을
채우는 데 필요한 점 수를 상한으로 씌운다. 폭 조건과 같은 것을 요구하는
셈이라 기준이 하나로 모이고, **고정값보다 느슨해질 때만** 적용되므로
근거리 잔차 억제력은 그대로다.

> 벽 팽창을 줄이면 벽 잔차가 늘어난다. FGM 오작동이 다시 보이면 여기부터
> 의심할 것. M-of-N(4/6, 5.3.8), near-wall 가드, 그리고 레이스라인 코리도
> 필터(`corridor_max_lateral_from_raceline_m`) 가 2차 방어선이다. 벽 잔차는
> 정의상 라인에서 멀어서 마지막 필터에 대부분 걸린다.

**(1-a) 스캔 시각으로 TF 를 조회한다**

위 예산에서 시각 불일치 항이 빠진 건 `_lookup_laser_to_map` 을 고쳤기
때문이다. 예전에는 `rclpy.time.Time()` — 즉 **최신 TF** 로 스캔을 변환해서,
스캔 시각과 어긋난 만큼 점구름 전체가 밀렸다.

| 불일치 | 2 m/s | 4 m/s | 7 m/s |
|--------|-------|-------|-------|
| 10 ms | 2.0 cm | 4.0 cm | 7.0 cm |
| 20 ms | 4.0 cm | 8.0 cm | **14.0 cm** |

코너(요레이트 1 rad/s)에서는 회전 성분이 더해져 20 ms 에 8 m 앞 점이
16 cm 옆으로 간다. `0.42` 가 필요했던 주된 이유가 이것이었다.

이제 `msg.header.stamp` 로 조회하고, 버퍼가 그 시각을 못 담으면 최신 TF 로
대체하되 **횟수를 세서 5 초마다 경고**한다. 이 경고가 자주 뜨면 반경을
낮춰 둔 근거가 사라진 것이니 되돌려야 한다.

**(2) 레이저 프레임 직선 튜브 — `obstacle_lateral_abs_max_m`**

플래너 게이트의 `|y| ≤ 0.42` 는 **차 진행축 기준 직선 튜브**다. 곡선이나
헤딩 오차가 있으면 레이스라인 정중앙 장애물도 튜브 밖으로 나간다.

| 상황 | 3 m 앞 | 4 m 앞 | 5 m 앞 |
|------|--------|--------|--------|
| 헤딩 5° 틀어짐 | 0.26 | 0.35 | **0.44 기각** |
| 반경 10 m 코너 | **0.45 기각** | **0.80 기각** | **1.25 기각** |
| 반경 15 m 코너 | 0.30 | **0.53 기각** | **0.83 기각** |

회피 시작 거리가 `avoid_on_m`(3.5) × 속도 스케일이라 보통 4.5 m 이상인데,
정작 그 거리에서 장애물이 사라진다.

레이스라인 코리도(`corridor_max_lateral_from_raceline_m`)는 **맵 좌표**로
재므로 곡선에서도 정확하다. 그래서 코리도가 도는 동안에는 튜브를
`obstacle_lateral_abs_max_corridor_m`(**1.50**) 로 넓혀 sanity bound 로만
쓰고 판단을 코리도에 맡긴다. 코리도를 못 쓸 때(TF 실패/비활성)만 예전의
좁은 튜브로 보수적으로 막는다. 코리도를 이미 통과한 목록에 튜브를 다시
거는 하류 지점(`_planner_gate_closest_m`, `_planner_closest_obstacle_m`)도
같이 넓혔다 — 방향 판정은 `forward_cone_deg`(75°) 가 한다.

코리도 검사는 이제 **장애물 반경을 뺀다** (`lat − r > corridor_max_lat_m`).
발행 좌표가 클러스터 최근접점이라 물체의 한쪽 끝이어서, 반경을 안 빼면
레이스라인을 물고 있는 50 cm 상자도 중심이 밖이면 통과시켜 버렸다.

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

#### 갭 **선택** 단계에도 차폭을 건다 (20260822 실측)

위 검증은 **이미 고른 갭 안에서만** 각도를 옮긴다. 그래서 고른 갭 자체가 못
지나갈 통로면 빠져나갈 길이 없다 — 각도로만 고르니 "넓어 보이는데 차폭은 안
들어가는" 갭이 이기고, 그 뒤에 아무리 훑어도 그 안은 전부 막혀 있다.

실측에서 AEB 까지 간 조우가 정확히 이 모양이었다.

```
t=17.0s  v=4.0  장애물 4.09 m  AVOID
gap 은 열렸지만 차폭이 안 들어감 — aim=-30° clear=0.57m < 1.00m
```

4 m/s 로 4.1 m 앞 박스를 만난 차가 −30° 를 겨눴는데 그 방향 여유가 0.57 m 다.
반대쪽이 열려 있어도 고를 수가 없다.

이제 `_gaps_that_fit()` 이 후보 갭마다 최선의 여유(`_gap_best_clear_m`)를 재서
`gap_fit_min_m` 에 못 미치는 갭을 **선택 전에** 뺀다. 안 뺐으면 히스테리시스가
그 갭에 계속 붙어 있었을 것이다.

| 파라미터 | 기본 | 설명 |
| --- | --- | --- |
| `gap_fit_check_enable` | `True` | 끄면 각도 폭으로만 고름 (예전 동작) |
| `gap_fit_samples` | `5` | 갭당 훑을 각도 수. 합격이 확인되면 조기 종료 |
| `gap_fit_min_m` | `1.0` | 이만큼도 안 뚫렸으면 후보에서 뺌 |

두 경우엔 손대지 않는다. **후보가 하나뿐이면** 걸러 봐야 고를 게 없고,
**전부 떨어지면** 원래 목록을 그대로 쓴다 — 못 지나가는 상황에서 판단을 바꾼다고
나아질 게 없고, 그때는 회피 순항속도와 AEB 가 받는다.

#### 판정은 "겨눌 수 있는 범위" 로 한다

처음 넣었을 때 경고가 그대로 나왔다. 3 바퀴에 6 번, 고친 적 없는 것처럼.

원인은 **재는 범위와 쓰는 범위가 달랐던 것**이다. 조준 각도는 갭 그대로가
아니라 가장자리 여유(`gap_edge_inset_rad`)를 물리고 탈출 콘과 속도 연동 FOV
로 또 잘린 범위에서 고른다. 그런데 적합성은 갭 **원래** 폭으로 쟀다. 갭 바깥쪽
25° 지점이 뚫려 있으면 합격인데, 정작 조준 단계에서 그 각도가 FOV 밖이라
쓰지 못한다. 걸러 놓고도 같은 데서 막히는 이유였다.

`_aim_range()` 로 그 좁히기를 한 군데 모으고 양쪽이 같이 부르게 했다. 이제
"들어간다" 는 판정은 실제로 겨눌 수 있는 각도에 대한 판정이다.

비용은 스캔당 갭 수 × 최대 5회의 코리도 계산인데, 합격이 확인되는 즉시 끊으므로
열린 갭은 보통 1회에 끝난다.

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
| `avoid_a_accel_mps2` | `4.0` | 회피 해제 후 **속도 회복** 기울기 상한 (0 이면 무제한) |
| `avoid_safety_factor` | `0.7` | 낮출수록 전체적으로 느리고 안전 |
| `avoid_standoff_m` | `0.35` | 장애물 앞 최소 이격 |
| `avoid_speed_min_mps` | `0.6` | 이 아래로는 안 줄임 (기어가지 않게) |
| `avoid_pass_clear_extra_m` | `0.15` | "지나갈 수 있다" 면제에 추가로 요구할 횡여유 |
| `deviation_speed_enable` | `True` | 이탈량 연동 감속 (아래 참고) |
| `deviation_speed_free_m` | `0.35` | 이 CTE 이하에서는 감속 없음 |
| `rejoin_a_lat_mps2` | `4.0` | 재합류 기동에 허용할 횡가속도 |
| `rejoin_max_heading_deg` | `18.0` | 합류각 상한 (저속쪽). 아래 참고 |
| `rejoin_min_heading_deg` | `10.0` | 합류각 하한 (고속쪽) |
| `rejoin_merge_overshoot_m` | `0.30` | 합류 시 라인을 넘어가도 좋은 양 |
| `rejoin_track_lag_s` | `0.30` | 추종 지연. 오버슈트 ≈ `v·sin(ψ)·τ` |
| `rejoin_max_path_curvature` | `1.19` | 조향 한계 `tan(21.4°)/0.33`. 넘으면 포기 |
| `rejoin_max_length_m` | `10.0` | 재합류 길이 상한 (트랙 둘레 41 m 의 24%) |
| `rejoin_stall_speed_mps` | `0.25` | 이 아래는 "안 움직이는 것" 으로 본다 |
| `rejoin_stall_sec` | `1.0` | 이만큼 정지하면 `REJOIN` 포기 |
| `rejoin_max_active_sec` | `5.0` | `REJOIN` 시간 상한의 **하한** (경로 길이 따라 늘어남) |
| `avoid_speed_ref_mps` | `2.0` | CSV 에 속도 열이 없을 때의 기준속도 |

> **회피 중 "지나갈 수 있는" 장애물은 정지거리 한계를 면제한다**: 정지거리
> 한계는 `gap = x − r − ego_front(0.50) − standoff(0.35)` 가 0 이하면 계산을
> 포기하고 `v_min`(0.6 m/s) 을 돌려준다. 정면 충돌만 가정한 판정이라, 옆으로
> 비켜 지나가는 중에도 그대로 걸렸다 — 실측에서 회피 내내 배율이 0.17 에
> 붙어 **0.49 m/s 까지 기어가다가** 장애물이 시야에서 빠지는 순간 다시
> 튀어 나갔다. 사용자가 본 "회피 중 차가 한 번 멈추고 갑자기 복귀" 다.
>
> 지금은 실제로 회피 조향 중일 때(`include_maneuver`) 기동을 끝냈을 때의
> 최소 횡거리(`passing_clearance_m`)를 재서, `r + 반폭 + 마진 +
> avoid_pass_clear_extra_m` 이상이면 정지거리 한계를 걸지 않는다. **부호를
> 본다** — 왼쪽으로 트는데 장애물이 오른쪽이면 멀어지지만 왼쪽이면 다가가므로,
> `|y|` 만으로는 구분이 안 된다. 면제돼도 조향 횡가속도 한계(maneuver)는
> 그대로라 속도가 무제한이 되지는 않는다.
>
> 같은 기하로 재현한 결과 (CSV 3.5 m/s, 왼쪽 1.0 m 회피):
>
> | 장애물 (x, y) | 수정 전 | 수정 후 |
> |---|---|---|
> | (2.0, −0.35) | 1.85 m/s `static` | 1.85 m/s `static` |
> | (1.6, −0.55) | 1.49 m/s `static` | 1.49 m/s `static` |
> | (1.2, −0.70) | 1.03 m/s `static` | **2.21 m/s** `maneuver` |
> | (0.9, −0.80) | **0.60 m/s** `static` | **2.21 m/s** `maneuver` |
>
> 접근 구간(`GLOBAL`)에서는 면제하지 않는다. 거기서는 레이싱라인이 굽어
> `|y|` 가 커 보일 뿐이라, 면제하면 코너 앞 콘을 CSV 전속으로 들이받는다.

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

> **`offset` 기동에서는 이 절이 거의 안 돈다.** 복귀 S커브가 기동 안에 들어
> 있어서, 기동이 끝나면 차는 이미 레이스라인 위에 접선으로 누워 있다. 그래서
> `AVOID → GLOBAL` 로 **바로** 간다. `REJOIN` 은 이제 AEB 탈출 뒤처리와
> FGM 폴백 경로를 탄 뒤처럼 **기동 밖에서 라인을 벗어난 경우**에만 남는다.
>
> 아래 내용은 그 잔여 경로에 대한 설명이다.

돌아온다. `rejoin_enable` 이 **기본 `True`** 라, 현재 위치에서 레이스라인까지
Frenet quintic 복귀 경로를 그려 `REJOIN` 모드로 붙는다. 레이스라인이 벽에
붙어 있는 구간에서는 이게 없으면 안 된다 — Stanley 피드백만으로 붙으면
접근각이 선 채로 라인을 만나 벽에 꽂힌다. quintic 은 끝점에서 `d′=d″=0`
이라 라인에 **접선으로 눕혀** 붙는다.

순서는 이렇다.

1. 장애물이 `avoid_fgm_gate_m` 밖으로 나가고 `_avoidance_fully_cleared` 가
   `avoid_off_count_th`(3) 사이클 연속 성립.
   그 디바운스 동안에도 override 는 **안 내린다** (`_publish_rejoin_bridge`)
2. 현재 (s, d, d′) 에서 d → 0 으로 가는 quintic 을 생성.
   길이는 **속도·이탈량·합류각·현재 헤딩** 연동 (`_rejoin_length_for`):
   `L = clip(max(rejoin_time_sec × v_ego, √(5.77·|d0|·v² / rejoin_a_lat_mps2),
   1.875·|d0| / tan(ψ)), rejoin_min_length_m, rejoin_max_length_m)`
   `ψ` 는 허용각 `ψ_max(v)` 와 **지금 이미 서 있는 각** 중 큰 쪽이고,
   라인을 향해 달리는 중이면 넘어감 상한이 한 번 더 자른다 (아래 절 참고)
   여기까지는 **기준선이 직선이라는 가정** 이고, `_plan_rejoin` 이 이 값을
   출발점으로 실제 코너 곡률을 보고 다시 고른다 (아래 두 절 참고)
3. 그 경로도 `_truncate_path_at_collision` 을 통과해야 채택된다.
   막히거나, **코너에서 Frenet 이 접히거나**, **라인을 넘지 않고 붙을 길이가
   없으면** `REJOIN` 을 포기하고 바로 `GLOBAL` — CSV 로 두는 편이 안전하다
4. 유지 중 경로는 **다시 그리지 않는다** (아래 참고)
5. 속도 상한은 경로의 **실측 초과곡률** 에서 역산한다:
   `v ≤ √(rejoin_a_lat_mps2 / (κ_path − κ))`. 경로가 없는 상태(GLOBAL/AVOID)
   에서는 아직 잰 값이 없으므로 이탈량 기반 `v ≤ L·√(a_lat/(5.77·|d|))` 을 쓴다.
   회복 기울기는 `avoid_a_accel_mps2` 가 묶는다
6. `|CTE| ≤ rejoin_finish_lateral_m` 이 되면 `REJOIN → GLOBAL`
7. 완료가 안 와도 **정지 `rejoin_stall_sec`(1.0 s)** 또는 **시간 초과** 면
   포기하고 `GLOBAL`. 시간 상한은 `max(rejoin_max_active_sec, 2.5 × L/v_계획)`
   으로 경로 길이를 따라간다 — 고정 5 s 로 두면 합류각 제약으로 길어진
   경로(저속·큰이탈에서 10 m)가 **완료 직전에** 끊기고, 그 순간 override 가
   내려가며 막으려던 급조향이 정확히 그때 나온다
8. `REJOIN` 중에도 `fgm_enable` 은 **켜 둔다**. 연속 장애물 구간에서 복귀
   도중 다음 게 나타나면 디바운스 없이 즉시 `AVOID` 로 돌아가는데, FGM 이
   꺼져 있었으면 목표점이 묵은 값이라 한두 프레임 헛돈다

> **재합류 경로를 다시 그리면 안 된다**: 일정 거리마다 현재 위치에서 다시
> 그려 봤더니 복귀가 오히려 거칠어졌다. 차가 경로를 못 따라가 벌어지는 중에
> 재생성이 겹치면 Stanley 기준경로가 통째로 갈아치워진다 — 실측 CTE 가
> `−0.11 → −0.43 → −0.64` 로 벌어지다 재생성 순간 `+0.01` 로 리셋되기를
> 0.3 초 주기로 반복했다. **추종 오차를 경로를 바꿔서 없애면 안 된다.**
> 그건 오차를 지우는 게 아니라 기준을 지우는 것이다. 기동은 1~2 초면
> 끝나므로 묵을 일도 없고, 차가 서서 오래 붙들리는 경우는 7번이 끊는다.

> **속도 상한에 최대 길이를 쓰면 안 된다**: `_deviation_speed_limit` 이
> `rejoin_max_length_m` 로 역산하면 이탈 1.6 m 에서도 상한이 전속 위라
> 사실상 안 걸린다. 그러면 복귀 내내 CSV 전속이 나가 라인에 세게 꽂힌다.
> 실제 경로 길이(예: 4 m)를 쓰면 같은 이탈에서 2.6 m/s 로 눌리고, 이탈이
> 줄면서 상한이 올라가 원래 속도로 자연스럽게 복원된다.

> **회피 해제 → 재합류는 기다리면 안 된다**: 예전에는 "CTE 가 줄 때까지"
> 최대 1.5 초 `AVOID` 를 붙들고 기다린 뒤에야 재합류 경로를 만들었다. 그런데
> 그 시점엔 발행 쪽이 이미 override 를 내려서 Stanley 기준경로가 로컬경로 →
> CSV 로 **순간이동** 한 뒤였다. 라인에서 벗어나 있으면 그 순간 CTE 가
> 계단으로 뛰고(실측 0.00 → −1.21 m) 급조향이 나간다. 기다리는 동안 보호받는
> 게 아니라 정확히 그 반대였다.
>
> 실측 타임라인 (회피 1회):
>
> | t | 사건 |
> |---|---|
> | 67.4~68.2 s | 회피 선감속으로 배율 0.43 → 0.17, **1.99 → 0.46 m/s (거의 정지)** |
> | 68.22 s | 배율 0.17 → 1.00 **한 프레임에 점프** |
> | 68.60 s | override 1 → 0 (mode 는 아직 `AVOID`) — CSV 로 생짜 전환 |
> | 68.6~69.5 s | 0.46 → 3.39 m/s, **최대 6 m/s²** 로 라인에 되꽂힘 |
> | 71.04 s | 그제서야 `REJOIN` 시작 — **2.4 초 늦음**, 그것도 v=0 에서 |
>
> 지금은 회피가 풀리는 즉시 재합류 경로를 깔고 override 를 유지한다. CTE 를
> 줄이는 건 재합류 경로가 할 일이지 기다린다고 줄어드는 게 아니다.
>
> **REJOIN 탈출 상한이 필요한 이유**: 완료 판정은 |CTE| 가 줄어야 성립하는데
> 차가 서 있으면 CTE 는 절대 줄지 않는다. 상한이 없던 시절 실측에서 정지 후
> **수 분간 `REJOIN` + `override=true`** 가 유지됐고, Stanley 는 수십 초 묵은
> 캐시 경로를 붙들고 있었다 — 다시 출발하면 과거 위치 기준 경로를 쫓는다.
> `AVOID → REJOIN` 대기에 `rejoin_wait_max_sec` 상한을 둔 것과 같은 이유다.
> 앞에 장애물이 남아 있으면 `avoid_on` 검사가 먼저라 `AVOID` 로 가므로,
> 이 탈출은 "회피할 게 없는데 갇힌" 경우에만 걸린다.

> **횡가속 예산으로 길이를 역산하는 이유**: quintic
> `d(s)=d0(1−10u³+15u⁴−6u⁵)` 의 최대 |d″| 는 `5.7735·|d0|/L²` 이고 요구
> 횡가속도는 여기에 `v²` 가 곱해진다. 시간에만 연동하고 `L` 을 2.50 m 로
> 자르면 7 m/s · 1.5 m 이탈에서 **67.9 m/s² (타이어 한계의 7배)** 를 요구한다.
> 추종 불가능한 경로라 조향만 포화된 채 벽으로 밀린다. 같은 조건에서
> 역산하면 `L = 10.3 m` 이고 요구 횡가속도는 예산인 4.0 m/s² 에 정확히 맞는다.
> 그래서 `rejoin_max_length_m` 기본값도 2.50 → **12.0** 으로 올렸다.
> (이후 합류각 제약을 넣으면서 트랙 둘레에 맞춰 **10.0** 으로 재산정)

### 합류각 — 라인에 비스듬히 꽂히지 않게 (`rejoin_max_heading_deg`)

횡가속도만 예산 안에 들면 되는 게 아니다. quintic 의 기울기 최대값이
`|d′| = 1.875·|d0| / L` 이고, 이게 곧 **복귀 경로가 레이스라인과 이루는 각**
이다. 실측 조건(이탈 1.2 m, 2.6 m/s)에서 시간·횡가속만 보면 `L = 3.4 m` 가
나오는데, 그 경로는 중간에서 라인과 **33°** 를 이룬다. 횡가속도는 통과지만
차는 라인을 향해 비스듬히 꽂히듯 들어가고, **레이스라인이 벽에 붙어 있으니
그 방향이 곧 벽 방향이다.** 추종이 조금만 늦어도 라인을 넘어 벽에 닿는다.

```
L ≥ 1.875 × |d0| / tan(rejoin_max_heading_deg)
```

각을 눕히는 방법은 길이뿐이다. 트랙 둘레가 41 m 라 무한정 늘릴 수 없어서
`rejoin_max_length_m`(10 m, 랩의 24%)에서 자른다.

**허용각은 속도에 따라 좁아진다.** 합류각 ψ 로 붙으면 라인을 가로지르는
속도성분이 `v·sin(ψ)` 라, 추종 지연 τ 가 같아도 빠를수록 더 많이 넘어간다:
오버슈트 ≈ `v·sin(ψ)·τ`. 각을 고정하면 고속에서 오버슈트가 속도에 비례해
커진다. 그래서 각이 아니라 **넘어가는 양** 을 예산으로 잡는다.

```
sin(ψ) ≤ rejoin_merge_overshoot_m / (v · rejoin_track_lag_s)
```

| 속도 | 목표각 | 이탈 1.2 m 의 길이 | 실제각 | 오버슈트 |
|---|---|---|---|---|
| 2.5 m/s | 18.0° | 6.9 m | 18.0° | 0.23 m |
| 4.0 m/s | 14.5° | 8.7 m | 14.5° | 0.30 m |
| 6.0 m/s | 10.0° | 10.0 m (상한) | 12.7° | 0.31 m |

고속쪽 하한을 10° 로 막아 둔 건 트랙 둘레 때문이다 — 그보다 눕히려면
만들 수 없는 길이가 필요하다. 그 대가로 6~7 m/s 에서 예산을 조금 넘긴다.

### 이미 라인을 향해 서 있을 때 — 길게 잡으면 거꾸로 간다

위 식(`1.875·|d0|/L`)은 차가 **라인과 나란할 때**(`d0′ = 0`) 의 것이다.
회피 직후는 정확히 그 반대다 — 이미 라인 쪽으로 비스듬히 달리는 중이고,
복귀 경로는 C1 연속이라 그 헤딩에서 **출발할 수밖에 없다.**

그 상태에서 허용각만 보고 길이를 늘리면, 초기 기울기가 라인을 지나쳐
**반대편으로 넘어갔다가** 되돌아오는 경로가 나온다. 벽에 붙은 라인에서는
넘어간 만큼이 그대로 반대쪽 벽이다. 넘어가는 양은 정규화하면

```
넘어감 = |d0| · F(p) ,   p = d0′ · L / d0
```

로 **`p` 만의 함수** 라 길이에 비례해 커진다. 즉 이 기동에서는 "길게 잡으면
완만해진다" 가 성립하지 않는다. 실측 조건(이탈 1.0 m, 헤딩 30°, 3 m/s)에서
허용각 12.8° 로 잡은 `L = 8.2 m` 는 라인을 **0.25 m** 넘어간다.

고친 건 세 가지다.

1. **`ψ` 를 "허용각과 이미 선 각 중 큰 쪽" 으로.** `L = 1.875·|d0|/tan(ψ_now)`
   는 `p ≈ −1.875` 라 넘지 않으면서 헤딩이 자연스럽게 눕는 길이다. 같은
   조건이 `L = 3.3 m` / 넘어감 0 이 된다. 라인에서 **멀어지는** 중이면
   (`d0·d0′ ≥ 0`) 경로가 각을 먼저 되돌려야 하므로 예전대로 허용각을 쓴다
2. **넘어감을 진짜 상한으로.** 위 셋(`l_time`/`l_accel`/`l_heading`)은 전부
   하한이라 제일 긴 놈이 이긴다. 고속에서는 `l_accel` 이 이겨서 다시
   넘어간다(0.6 m / 30° / 6 m/s 에서 5.6 m 를 요구, 0.21 m 넘어감).
   그래서 `_rejoin_crossing_cap_m` 이 예산의 95 % 를 목표로 이분해 자른다.
   **넘어감이 이긴다** — 횡가속 예산은 속도로 갚을 수 있지만 벽은 못 갚는다
3. **`_plan_rejoin` 이 후보마다 실제로 잰다** (`_rejoin_line_crossing_m`).
   순위는 위험한 순서다: 라인을 넘는 후보(tier 2) → 못 따라가는 후보
   (tier 1) → 둘 다 통과(tier 0, 여기서 직선 가정값에 가장 가까운 걸 선택)

**남은 게 tier 2 뿐이면 만들지 않는다.** 급코너에 비스듬히 서면 짧게는
조향 한계를 넘고 길게는 라인을 넘어 **사이에 답이 없는** 경우가 있다.
그때는 무한대를 돌려 호출부가 포기하게 하고 CSV 를 유지한다. Stanley 의
피드백은 접지력 예산에 묶여 있어서, 벽을 향하는 경로를 쥐여 주는 것보다
낫다.

반경 1.41 m 짜리 극단 코너 픽스처로 576 조합을 훑은 결과:

| | 이전 | 지금 |
|---|---|---|
| 경로를 냄 | 472 | 448 |
| 그중 예산 이상 넘어감 | **182** | **0** |
| 포기 (CSV 유지) | 104 | 128 |

넘어가던 182 건이 0 이 되는 대가로 24 건이 CSV 로 넘어간다.
(`test_rejoin_heading_aware.py`)

### 코너에서의 재합류 — 기준선 곡률 (`_build_curvature`, `_plan_rejoin`)

복귀 경로는 기준선에서 `d` 만큼 떨어진 오프셋 곡선이고, 그 곡률에는 기준선
곡률이 **`1/σ` 로 증폭돼** 들어간다 (`σ = 1 − d·κ`). 코너 **안쪽** 으로
벗어나 있으면 `d·κ > 0` 이라 분모가 작아진다. `d·κ → 1` 이면 서로 다른 `s`
의 법선이 곡률 중심에서 만나 오프셋 좌표계 자체가 **접힌다**.

이 트랙은 둘레 41 m 에 최소반경 **1.54 m**, 상위 25% 가 R ≤ 3.3 m 다.
직선 가정(`5.77·|d0|/L²`)만으로 길이·속도를 정하면 이걸 통째로 놓친다 —
실측으로 이탈 1.5 m 로 헤어핀 안쪽에 있으면 요구 횡가속도가 **예산의 17 배**
인데 직선 가정은 "예산 안" 이라고 답했다.

`_build_curvature` 가 CSV 적재 시 `κ(s)`, `κ′(s)` 를 미리 깔고
(폴리라인 간격 0.05 m 는 인접 3점으로 재면 노이즈뿐이라 **±1 m 베이스라인**),
`_plan_rejoin` 이 길이 후보를 훑으며 Frenet→직교 곡률식으로 실제 값을 잰다.

```
κ_path = ((d″ + (κ′d + κd′)·tanΔθ)·cos²Δθ/σ + κ)·cosΔθ/σ ,  Δθ = atan2(d′, σ)
```

> **재는 건 `κ_path` 가 아니라 `κ_path − κ` 다**: 기준선 곡률만큼은 레이스라인을
> 그냥 달려도 어차피 감당하고 CSV 속도가 이미 그걸로 짜여 있다.
> `rejoin_a_lat_mps2`(4.0)는 복귀 기동이 **추가로** 만드는 몫의 예산이라
> 총량과 비교하면 안 된다 — 총량으로 재면 중간 코너(κ≈0.16)만 지나도
> `v²κ = 5.8` 이라 직선 복귀조차 3.8 m/s 로 묶인다. 기준선이 직선이면
> 이 값은 예전 `5.77·|d0|/L²` 로 그대로 환원된다 (테스트로 고정).

**포기해야 하는 세 경우** — 전부 감속으로 해결되지 않는다:

1. `σ ≤ 0.25` : 좌표계가 접히기 직전이라 곡률이 발산한다
2. `|κ_path| > rejoin_max_path_curvature`(1.19) : 조향 한계를 넘는다.
   초과분만 보면 안 걸린다 — 이미 급한 코너에서는 조금만 더 휘어도 핸들이
   끝까지 돌아간 상태가 되기 때문이다
3. 모든 후보가 라인을 `rejoin_merge_overshoot_m` 이상 넘는다 : 위 절 참고

포기하면 `REJOIN` 을 접고 CSV 로 넘긴다. Stanley 의 피드백은 접지력 예산에
묶여 있어서, 깨진 경로를 주는 것보다 이쪽이 안전하다. 실측 포기율은 이탈
1.2 m 에서 4%, 1.5 m 에서 8% 이고 전부 헤어핀 정점 부근이다.

속도 상한도 잰 값에서 나온다: `v ≤ sqrt(rejoin_a_lat_mps2 / (κ_path−κ))`.
이탈 1.2 m 기준 상한 중앙값 6.0 m/s, 86% 구간에서 4 m/s 이상이 나온다.

> **길어진 경로는 느려지는 게 아니라 빨라진다**: 요구 횡가속도는 `1/L²` 로
> 떨어지고 이탈량 연동 감속 상한은 `L` 에 비례해 올라간다. 이탈 1.2 m /
> 6 m/s 에서 `L` 이 7.9 → 8.4 m 로 늘면 요구 횡가속도가 4.0 → 3.5 m/s² 로
> 내려간다. 완만하게 붙는 쪽이 더 빠르다.

> **점 개수가 아니라 간격을 고정한다**: `first_blocked_index` 는 경로 점만
> 검사하고 사이를 보간하지 않아서 점 간격이 곧 충돌 검사 해상도다. 개수를
> 30 으로 고정하면 `L` 이 길어질수록 간격이 벌어져(10 m/30점 = 0.34 m)
> 얇아진 벽 팽창대를 그대로 건너뛴다. 꼬리와 같은 0.05~0.1 m 간격으로 깔고
> `rejoin_sample_count` 는 하한으로만 쓴다.

### 이탈량 연동 감속 (`_deviation_speed_limit`)

`rejoin_max_length_m` 안에 붙을 수 있는 속도로 상한을 건다.

```
v ≤ rejoin_max_length_m × √(rejoin_a_lat_mps2 / (5.77 × |CTE|))
```

`deviation_speed_free_m`(0.35 m) 이하에서는 상한이 무한대라 정상 주행에는
걸리지 않는다. 3 m 벗어나면 5.8 m/s 로 묶인다 — AEB 나 큰 회피로 경로에서
크게 벗어난 채 전속을 유지하다가 복귀 시점에 조향이 포화되는 것을 막는다.
`avoid_speed_min_mps` 아래로는 내려가지 않아서 갇히지 않는다.
`avoid_speed_enable=False` 면 이 상한도 함께 꺼진다.

### 좁은 데 들어온 차에게 "거기서 나가는 길" 을 거부하고 있었다

5~6 바퀴에 벽을 세 번 쳤다. 나머지도 "조마조마" 했다고 한다. 로그에는
`REJOIN 포기` 가 일곱 번 찍혀 있었는데, 전부 같은 모양이었다.

```
REJOIN 포기 — 1번째 점에서 막힘. 이탈 +0.22 m, 헤딩 +13°, v=3.7
REJOIN 포기 — 6번째 점에서 막힘. 이탈 +0.23 m, 헤딩 +17°, v=3.1
REJOIN 포기 — 8번째 점에서 막힘. 이탈 +0.35 m, 헤딩  -4°, v=4.7
...
```

헤딩은 21° 이내고 이탈도 0.35 m 이하다. 점 간격이 0.055 m 이므로 막힌 자리는
차 앞 **5~55 cm** 다. 경로가 나쁜 게 아니라 **그 자리 자체가 막힌 것으로
판정** 되고 있다는 뜻이다.

`debug/raceline_clearance.py` 로 재 봤다.

```
레이스라인 → 벽 여유:  최소 0.316 m,  중앙 0.622 m
팽창반경 0.254 m

라인 자체        : 0 점 (0.0 %)   — 전 구간 통과
이탈 0.20 m 면   : 189 점 (25.2 %) 가 팽창대 안
이탈 0.35 m 면   : 354 점 (47.2 %) 가 팽창대 안
```

라인 위는 늘 통과한다. 그런데 **30 cm 만 비키면 트랙의 절반이 막힌다.**
회피란 곧 옆으로 비키는 것이므로, 회피한 차는 절반의 확률로 "막힌" 자리에
서 있게 된다.

거부해도 차는 그 자리를 벗어나지 못한다. 오히려 재합류 경로가 **바로 그
팽창대에서 빠져나가는 길** 이라, 거부가 정확히 반대로 작동한다. 그리고
거부의 대가는 override 해제 → 기준경로가 CSV 로 점프 → 급조향이다.

#### 팽창반경은 벽이 아니라 예산이다 (`_clearance_floor_at`)

`first_blocked_index` 의 주석에 이미 답이 있었다.

> `start_index`: 차량 현재 위치(0번)는 보통 건너뛴다 — 이미 거기 서 있는데
> "막혔다" 고 해봐야 할 수 있는 게 없고 …

논리는 맞는데 **점 하나(5.5 cm)** 에만 적용됐다. 차가 이미 들어와 있는 영역
전체에 적용돼야 한다.

이제 통과 기준을 호출부가 정한다. 차가 선 자리의 실제 여유를 읽어서

```
floor = max(차 반폭 0.15, min(팽창반경 0.254, 지금 여유))
```

를 넘긴다. 라인 위(여유 0.62 m)면 `min` 이 팽창반경을 고르므로 **아무것도
바뀌지 않는다.** 좁은 데 들어와 있을 때만 "지금만큼" 으로 풀린다. 반폭
밑으로는 절대 안 내려간다 — 거긴 마진이 얇은 게 아니라 실제로 못 들어간다.

더 파고드는 경로는 그대로 막힌다. 풀어 준 건 "지금보다 나빠지지 않는" 데까지다.

### 수직에 가까운 헤딩 — 투영이 45° 라고 거짓말하고 있었다

실차에서 회피 뒤 복귀하다 **두 바퀴 연속으로** 벽에 박았다. 운전자 증언이
결정적이었다: 박은 순간 차의 헤딩이 레이스라인과 거의 **수직** 이었고,
복귀 자체는 "천천히 붙고 있었다".

원인은 계획도 제어도 아닌 **좌표 변환** 이었다.

```python
d0p = math.tan(yaw_err)
d0p = max(-1.0, min(1.0, d0p))   # ← ±1.0 = tan(45°)
```

`tan` 의 발산을 막으려고 자른 것 자체는 맞다. 문제는 자른 값을 그대로
플래너에 넘기고, **잘렸다는 사실을 아무에게도 알리지 않은** 것이다. 75° 로
벽을 향한 차가 45° 로 들어갔다. 30° 의 차이가 통째로 사라졌다.

그 뒤는 전부 자신 있게 틀렸다. quintic 은 45° 를 시작 기울기로 얌전한 경로를
뽑고, `_rejoin_line_crossing_m` 은 그 경로가 라인을 안 넘는다고 확인해 주고,
`_truncate_path_at_collision` 은 벽에 안 닿는다고 통과시킨다. 세 검사가 전부
통과한다 — 차가 그 경로의 시작 방향을 30° 벗어나 있다는 것만 빼고. 차는
자기가 향한 곳으로 갔고, 거기가 벽이었다.

**한계를 다루는 것과 한계를 숨기는 것은 다르다.** 지금은 유효 한계
(`rejoin_yaw_err_limit_deg`, 55°) 까지만 quintic 을 믿고, `yaw_err` 는 자르지
않은 원본을 같이 돌려준다. 호출부가 직접 보고 판단한다.

#### 한계를 넘으면: 복귀 대신 정렬 (`_build_alignment_path`)

수직에 가까운 차에게 "라인으로 돌아와라" 는 경로는 두 가지를 한꺼번에
시킨다 — 방향을 돌리는 것과 옆으로 붙는 것. 차는 앞의 것부터 할 수밖에
없는데 경로는 뒤의 것 기준으로 그려져 있으니, 추종 오차가 벌어지고 그
방향이 하필 차가 향하던 벽 쪽이다.

그래서 옆으로 붙는 요구를 **뺀다**. 지금 이탈량 `d0` 를 그대로 유지한 채
트랙 방향으로 나란히 가는 경로를 준다. CTE 항이 0 근처라 Stanley 에는 헤딩
항만 남고, 그게 곧 정렬이다. 방향이 한계 밑으로 들어오면 `_alignment_done`
이 이 경로를 버리고 정식 복귀를 다시 그린다. 이탈은 그때 줄인다.

CSV 로 넘기지 않는 이유는 `_publish_rejoin_bridge` 와 같다 — override 가
내려가는 순간 기준경로가 튀면서 CTE 가 계단으로 뛴다 (실측 0.00 → −1.21 m).
여기서는 override 를 쥔 채로 방향만 맞춘다.

정렬 경로는 "한 번 그리면 끝까지" 규칙의 유일한 예외다. 그 규칙이 막으려던
건 *추종 오차 때문에* 기준을 갈아치우는 것이지, 계획의 전제가 바뀐 경우가
아니다.

놓는 각(33°)은 진입각(55°)보다 낮다. 같은 문턱을 쓰면 경계에서 정렬↔복귀가
떨리고, 더 나쁘게는 딱 55° 에서 넘겨받은 quintic 이 `tan(55°)=1.43` 이라 대개
곡률 예산에 걸려 포기로 간다 — 문턱을 옮기기만 한 꼴이다.

| 파라미터 | 기본 | 설명 |
| --- | --- | --- |
| `rejoin_yaw_err_limit_deg` | `55.0` | 이 각을 넘으면 quintic 대신 정렬 |
| (해제각) | 진입각 × 0.6 | 33°. 받는 쪽이 실제로 풀 수 있는 각 |

#### 복귀 실패는 이제 항상 로그에 남는다

세 개의 재합류 포기 경로가 전부 `verbose_logs` 뒤에 있었다. 조용히 넘어갈
사건이 아니다 — **차가 라인에서 벗어나 있는 채로 override 가 내려가는**
순간이고, 그때부터 Stanley 가 CSV 를 직접 겨눈다. 부드럽게 붙이려고 만든 게
재합류인데, 그게 실패한 자리에서 가장 거친 복귀가 나온다.

`_warn_rejoin_given_up` 이 사유와 함께 이탈량·헤딩·속도를 찍는다 (1초 스로틀).

### 복귀가 라인을 넘어 벽으로 가는 두 경로

실주행 증상은 둘이었다. **(1)** 복귀가 코너에 겹치면 라인을 가로질러 바깥
벽으로 가고, **(2)** 벽에 가까운 라인으로 복귀할 때 관성으로 넘어가 박는다.
직선 구간이 길면 아무 문제가 없었다.

원인도 둘인데 서로 다른 층에 있다. 하나만 고치면 나머지 절반이 남는다.

#### (1) 계획층 — 기동이 기준선을 직선으로 가정했다 (`avoid_offset_corner_aware`)

`plan_maneuver` 의 예산 검사가 기동 **자신의** `|d''|` 만 봤다. 코너에서는
기준선이 이미 `v²κ` 를 쓰고 있어서 실제 접지력 부하는 둘의 합이다.

```
a_total ≈ v²·(|κ| + |d''|)
```

R=6 m 를 6 m/s 로 도는 것만으로 6.0 m/s² 다. 접지력은 5~6 (IMU 실측: 2.5~2.8
m/s 코너링에서 v·ω 피크 4.84~5.59 에서 이미 밀림) 이라 복귀에 쓸 몫이 없다.
그런데 `d''` 만 보는 검사는 `peak_a = 3.0` 이라고 보고하고 통과시켰다. 감속도
계획 실패도 안 났고, 차는 조향을 물고 라인을 가로질러 나갔다.

이제 `kappa_ref` 를 넘겨 구간별 `|κ|` 를 예산에서 **먼저 빼고** 길이를 뽑고,
조향 한계와 속도 상한도 `|κ|+|d''|` 로 판단한다. 순서는 "먼저 완만하게, 그래도
안 되면 감속" 이다 — 복귀 길이를 늘려 보고, 그래도 `a_lat_hard`(4.5)를 넘으면
`speed_cap_mps` 로 답한다.

| v | 기준선 | 복귀 길이 | `peak_a` | 속도 상한 |
| --- | --- | --- | --- | --- |
| 6.0 | 직선 | 8.66 m | 3.00 | — |
| 6.0 | R=10 | 12.0 m | 4.54 | 5.98 m/s |
| 6.0 | R=8 | 12.0 m | 5.44 | 5.46 m/s |
| 7.0 | 직선 | 10.11 m | 3.00 | — |
| 7.0 | R=6 | 12.0 m | 9.44 | 4.83 m/s |

**직선은 한 자리도 안 바뀐다.** 회피를 느리게 만드는 게 목적이 아니라 코너와
겹칠 때만 개입하는 것이다. 기존 오프셋 기동 테스트가 이 기능을 켠 채로 그대로
통과하는 것이 그 보증이다.

#### (2) 제어층 — 고속에서 복귀 감쇠를 깎고 있었다 (`stanley_heading_weight_speed_*`)

Stanley 에서 헤딩항은 오버슈트를 **만드는** 항이 아니라 **막는** 항이다.

```
δ = θ_e + atan(k·e/v)
```

`θ_e` 가 있어야 오차 동역학이 1차(`ė = −k·e`)가 되어 라인을 안 넘는다. 그걸
깎으면 2차 무감쇠계가 되어 지나친다. `stanley_heading_oppose_only_blend` 가
켜진 뒤로 억제가 걸리는 경우는 정확히 "헤딩항이 복귀를 되받는 중" 뿐이라,
남은 억제(`stanley_heading_min_weight` 0.15)는 **감쇠만 골라서** 깎고 있었다.

실제 `_stanley_control` 을 그대로 호출하는 폐루프 검사(접지력 6 m/s², 라인에서
0.5 m 벌어진 채 라인 쪽으로 각을 물고 도착). 값은 반대편으로 넘어간 거리 [m]:

| 도착각 | v=5 | v=6 | v=7 | → 수정 후 v=7 |
| --- | --- | --- | --- | --- |
| 15° | 0.000 | 0.000 | 0.000 | 0.000 |
| 20° | 0.000 | 0.006 | **0.102** | **0.000** |
| 25° | 0.033 | 0.155 | **0.336** | 0.263 |
| 30° | 0.094 | 0.303 | 0.591 | 0.483 |

넘어가는 건 **도착각과 속도**의 문제지 CTE 크기가 아니다. 15° 이하는 어느
속도에서도 안 넘고, 20° 부터 7 m/s 에서 넘기 시작한다.

억제를 통째로 없애지는 않았다. 0.15 라는 값은 20260816 실측(2.5~3 m/s)에서
나온 것이고 저속에서는 넘지도 않는다. 오버슈트가 실제로 생기는 구간에서만
푼다 — `speed_lo`(4.0) 아래는 그대로, `speed_hi`(6.0) 위는 억제 없음, 사이는
선형보간. 4 m/s 이하 거동은 수치가 **비트 단위로 동일**한 것을 테스트로 묶었다.

25° 이상은 감쇠로 완전히 막지 못한다. 그건 감쇠가 아니라 **도착각**으로 풀
문제이고, REJOIN 은 이미 합류각을 속도로 묶는다(아래 `rejoin_max_heading_deg`,
`rejoin_merge_overshoot_m/rejoin_track_lag_s` → 횡접근속도 0.67 m/s 상한 →
7 m/s 에서 5.5°). 계획을 벗어난 각으로 들어오는 경로는 FGM 폴백 쪽이다.

근거 수치와 회귀는 `test/test_rejoin_overshoot.py` 가 배포 CFG 로 고정한다.

### 피드백 조향의 접지력 예산 (Stanley, `feedback_lateral_accel_mps2`)

위 두 가지는 플래너가 **경로와 속도**를 고쳐 주는 것이고, 마지막 안전망은
Stanley 쪽에 있다. 조향 명령을 두 몫으로 나눠 **피드백만** 묶는다.

```
δ = δ_ff(경로 곡률)  +  clip(heading + CTE + 가속보정,  ±atan(L·a_fb/v²))
```

FF 는 경로 자체의 곡률이고 그 곡률은 이미 속도 프로파일에 반영돼 있으니
자르면 안 된다 (자르면 코너에서 언더스티어가 난다). 반면 나머지 세 항은
"경로에서 벗어난 만큼" 커지는 항이라, 고속에서 크게 벗어나면 접지력을
넘는 조향을 만들어 낸다 — **7 m/s 에서 10° 만 꺾어도 26 m/s²** 다.

`max_lateral_accel_mps2`(총조향 상한)는 FF 를 자르는 문제 때문에
`LOCAL_PATH` 에만 걸려 있었고, 그래서 **CSV 추종에는 상한이 아예 없었다.**
`feedback_lateral_accel_mps2` 는 FF 를 건드리지 않으므로 두 모드 모두에 건다.
상한이 걸리면 세 항을 함께 비례 축소하므로 `/stanley_debug` 의 항별 값과
최종 명령이 계속 일치한다.

저속에서는 상한이 `max_steering` 보다 커서 사실상 안 걸린다
(2 m/s 18.3°, 5 m/s 3.0°, 7 m/s 1.5°). 작아 보이지만 7 m/s 에서 1.5° 가
정확히 4 m/s² 를 만드는 각도다 — 그 위는 명령해도 곡률로 바뀌지 않는다.

#### 계획 경로에서는 FF 를 켜야 한다 (`/planner/local_path_planned`)

위 상한은 **오차 보정**을 묶으라고 넣은 것이다. 그런데 `LOCAL_PATH` 에서는 FF 도
함께 꺼져 있었다. FGM 폴백 경로만 생각하면 맞다 — 조준점까지 그은 직선이라
곡률이 목표점 흔들림에서 나오는 잡음이고, FF 로 증폭하면 조향이 떤다.

`offset` 기동은 정반대다. 그 곡률은 우리가 타기로 **계획한** 값이다. FF 를 끄면
계획된 기하조차 피드백이 만들어야 하는데, 상한이 딱 그만한 크기다.

| v | 계획 곡률이 요구하는 조향 | 피드백 상한 | 상한 대비 |
|---|---|---|---|
| 4 m/s | 3.54° | 4.72° | 75% |
| 5 m/s | 2.27° | 3.02° | 75% |
| 6 m/s | 1.57° | 2.10° | 75% |
| 7 m/s | 1.41° | 1.54° | **91%** |

7 m/s 에서 오차 보정에 남는 건 상한의 9% 다. 계획 경로에서 벌어져도 되돌릴 힘이
없다는 뜻이고, 그게 "자율주행 라인을 제대로 안 따라간다"의 정체였다.

그래서 플래너가 `/planner/local_path_planned`(Bool) 로 **지금 주는 경로가 계획된
기하인지** 알려주고, Stanley 는 그때만 `LOCAL_PATH` 에서도 FF 를 켠다.

```
ff_ok = (mode != "LOCAL_PATH") or local_path_planned
```

| 경로 | planned | FF |
|---|---|---|
| `offset` 기동 | `True` | 켬 |
| FGM 폴백 (`straight`) | `False` | 끔 |
| `REJOIN` 복귀 경로 | `False` | 끔 (게인이 FF 없이 맞춰져 있음) |
| CSV 추종 | 해당 없음 | 항상 켬 |

이러면 역할이 제자리를 찾는다. **FF 가 계획된 곡률을 내고, 피드백 상한은 순수하게
오차 보정 크기를 묶는다.** 게이트가 끊기면 Stanley 는 어차피 CSV 로 떨어지므로
플래그가 `True` 로 굳어도 위험하지 않다.

`rejoin_enable=False` 로 끄면 2번 대신 그냥 `/planner_path_override_active` 를
내려서, Stanley 가 CSV 로 알아서 붙게 둔다. 기본값은 `True` 다.

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
| `AVOID → GLOBAL` | **기동 완료** (`ds ≥ total_length_m`) — 복귀가 기동에 들어 있어 `REJOIN` 을 건너뛴다 |
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

#### 한 프레임 실패로 회피를 접지 않는다 — `avoid_blocked_frames_th`

위의 시간 래치는 모드 **떨림**은 잡았지만, 반대 방향으로 과했다. 래치는
**첫 실패 한 번**에 걸리는데 걸리는 순간 0.5 초를 통째로 `GLOBAL` 로 보낸다.
회피 경로 생성은 FGM 조준·TF·잘림 판정이 겹쳐 있어 한 프레임쯤은 쉽게
실패하므로, 그 한 번이 0.5 초짜리 CSV 주행이 된다 — 5 m/s 면 장애물을 향해
**2.5 m 직진**이다. 실주행에서 "로컬패스로 갔다가 갑자기 글로벌패스로" 갈피를
못 잡는 것처럼 보이던 게 이것이고, 회피를 시작해 놓고 라인으로 돌아가니
안 피하느니만 못했다.

회피하기로 정했으면 붙들고 있어야 한다. 두 가지를 건다.

- **포기 판정에 연속 실패 프레임을 요구한다** (`_avoid_give_up`).
  `avoid_blocked_frames_th`(5) = 40 Hz 에서 125 ms. 진짜로 못 지나가는
  상황은 몇 프레임이면 확실히 드러난다. 성공하면 카운트는 0 으로 돌아간다.
- **접기 전까지는 직전 경로를 다시 낸다** (`_hold_last_avoid_path`).
  모드만 붙들고 게이트를 내리면 Stanley 는 그 즉시 CSV 로 돌아가서, 라벨만
  `AVOID` 인 채 장애물 쪽으로 되돌아간다. 직전 프레임에 충돌 검사를 통과한
  경로이므로 `avoid_hold_max_sec`(0.2 s) 안에서는 붙들 수 있다. 그보다
  오래된 경로는 차가 이미 지나쳐서 뒤를 가리키므로 안 쓴다.

포기한 뒤는 예전과 같다 — 앞차가 있으면 `TRAILING`, 없으면 `GLOBAL`. 그
사이 감속은 속도 정책이, 최후는 AEB 가 받는다.

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

### 저속에서는 로컬패스로 늦게 갈아탄다

`avoid_on` (CSV → `/local_path` 전환 거리)을 4 m/s 이하에서 **0.7 배**로
줄인다 (`_avoid_on_late_factor`). 저속에서 일찍 갈아타 봐야 장애물이 아직
멀어 FGM 조준이 흔들리고, 그동안 레이스라인만 놓친다.

**FGM 이 켜지는 거리(`fgm_enable_m`)는 안 건드린다.** 늦추는 건 갈아타는
시점뿐이라, FGM 은 예전처럼 미리 켜져서 갭을 계속 갱신하고 있다가 전환
순간 바로 쓸 수 있는 목표를 준다.

| v (m/s) | 전 | 후 | FGM 켜짐 |
|---|---|---|---|
| 1 | 2.80 m | 1.96 m | 5.20 m |
| 2 | 4.55 m | 3.18 m | 10.40 m |
| 3 | 6.83 m | 4.78 m | 12.00 m |
| 4 | 9.10 m | 6.37 m | 12.00 m |
| 5 | 11.38 m | 11.38 m | 12.00 m |
| 7 | 12.00 m | 12.00 m | 12.00 m |

고속은 손대지 않는다 — 거기서 늦추면 피할 거리가 안 나온다.
`avoid_on_late_blend_mps`(1.0) 구간에 걸쳐 원복해서 문턱에서 게이트가 튀지
않게 한다 (튀면 AVOID↔GLOBAL 이 떤다).

### 벽으로 나가는 회피 경로 — 설 수 있어야 받는다

레이스라인이 벽에 붙어 있는 구간에서 FGM 이 바깥쪽 갭을 고르면 회피 경로가
벽을 향한다. `_truncate_path_at_collision` 이 맵(`InflatedMap`)으로 벽을 보고
경로를 잘라 주기는 했는데, **잘린 경로를 받아들이는 기준이 고정 길이**
(`path_check_min_length_m`, 0.6 m)뿐이었다. 6 m/s 에서 0.6 m 는 0.1 초다 —
"쓸 만하다" 며 벽을 향한 경로를 내보내고, 차는 그 방향으로 돌았다.

이제 길이 기준을 속도로 만든다 (`_wall_stop_distance_m`). 잘린 끝이 벽이므로,
그 앞에서 설 수 있어야 그 경로를 받을 자격이 있다.

    필요 개방거리 = v·wall_stop_reaction_sec + v²/(2·avoid_a_brake_mps2)

| v (m/s) | 필요 개방거리 |
|---|---|
| 1 | 0.32 m |
| 3 | 1.95 m |
| 5 | 4.92 m |
| 7 | 9.22 m |

**이걸 기각 조건으로 쓰면 안 된다 (20260822 실측).** 원래는 모자라면 회피를
포기하고 CSV 를 유지했다. 논리는 "느려진 다음 주기에는 같은 경로도 통과한다"
였는데, 실차에서는 그렇게 안 굴러갔다.

한 바퀴에 세 번 걸렸고 **세 번 다 벽에서 잘렸다.**

```
원인=벽 (벽 17, 장애물 48)  확보 2.10 m < 요구 2.14 m  (v=3.2) FGM=+25°
원인=벽 (벽 10, 장애물 42)  확보 1.05 m < 요구 1.09 m  (v=2.2) FGM= -1°
원인=벽 (벽 21, 장애물 55)  확보 2.70 m < 요구 4.18 m  (v=4.6) FGM= +0°
```

장애물 쪽 절단 인덱스가 훨씬 뒤다 — **장애물은 제대로 피하는 경로** 였다.
그런데 둘은 요구치에 **4 cm** 모자라서 버려졌다.

버린 뒤가 문제다. 회피에 들어간 이유가 "레이스라인 위의 장애물" 이므로
CSV 는 **정의상 그 장애물을 향한다.** 짧아도 비켜 가는 경로를 버리고 정면으로
가는 경로를 택하는 셈이라, 기각이 곧 충돌 코스다. 게다가 요구치가 `v²` 로
자라서 빠를수록 더 긴 확보를 요구한다 — 정작 급할 때 제일 엄격해진다.
직전 경로를 붙드는 장치(`_hold_last_avoid_path`)도 **기동 첫 프레임에는 붙들
직전 경로가 없어서** 못 막는다. 실측 흐름이다.

```
69.09s  검출 4.4m → AVOID, FGM 켜짐
69.26s  경로 막힘 → 회피 포기, override=0
69.33s  모드 → GLOBAL   (path_n=203 = CSV, 박스로 직진)
69.73s  AEB (1.6m)
69.83s  override 겨우 1  ← 최초 AVOID 로부터 740 ms
```

지켜야 할 명제는 **"확보한 길이 안에서 설 수 있어야 한다"** 이지 "설 수 없으면
그 경로를 쓰지 말라" 가 아니다. 전자는 속도로 지킬 수 있다. 그래서 지금은
확보 길이를 **속도 상한**으로 바꾼다 (`_cleared_path_speed_limit`, 위 식의
역함수).

    v_max = −a·t + √((a·t)² + 2·a·L)

| 확보 L | 그때 속도 | 새 상한 |
|---|---|---|
| 2.10 m | 3.2 | **3.13** |
| 1.05 m | 2.2 | **2.10** |
| 2.70 m | 4.6 | **3.60** |

세 건 다 경로가 통과하면서 감속도 그대로 일어난다. 상한은 회피 순항속도보다
**뒤에** 걸린다 — 덮어쓰기가 아니라 안전 상한이라, 순항속도라도 이걸 넘을 수는
없다. 경로가 안 잘렸으면 `inf` 라 아무 일도 안 하고, `_publish_override_gate(False)`
에서 풀려서 회피가 끝나면 CSV 속도로 돌아간다.

남은 기각 조건은 고정 하한(`path_check_min_length_m`, 0.6 m) 하나다. 그보다
짧으면 Stanley 가 끝점에서 이상하게 도는 게 실제 문제라 그대로 둔다. AEB 탈출
중에는 호출부가 `min_length_m` 을 따로 넘겨 더 짧은 경로를 허용한다.

끄려면 `wall_stop_check_enable:=false` (상한까지 같이 꺼진다).

### 회피 순항속도 — 고속 3, 저속 2

**진입 속도로 가른다.** 저속에서 만나는 장애물은 대개 코너나 좁은 구간이라
같은 회피라도 여유가 적다 — 거기서만 한 단 더 깎는다.

| 진입 속도 | 순항속도 |
|---|---|
| > `avoid_cruise_high_speed_th`(4.0) | `avoid_cruise_speed_high_mps` **3.0** |
| ≤ 4.0 | `avoid_cruise_speed_low_mps` **2.0** |

위의 물리 한계들(maneuver/static/dynamic/deviation)은 각자 다른 걸 본다 —
조향 가능성, 정지거리, 이탈량. 구간마다 다른 놈이 이기면 접근에서 한 번,
회피 중에 또 한 번, 복귀에서 다시 속도가 바뀐다. 특히 거리 기반 항은
장애물이 가까워질수록 계속 낮아져서, 정작 피하는 순간에 제일 느리다.

그래서 이 값은 **상한이자 하한**이다. `_planner_speed_scale` 맨 끝에서
덮어써서 접근·회피·복귀가 같은 속도가 된다. 더 낮게 부른 이유가 있어도
되돌린다 — **급감속은 AEB 뿐이다.** 예외는 둘. `TRAILING`(앞차 속도를 따라야
한다)과 AEB 탈출(`aeb_escape_speed_mps` 0.8, 기어 나가는 동작).

**한 번 고르면 붙든다** (`_avoid_cruise_target`). 여기서는 래치가 편의가
아니라 필수다 — 고속 목표(3.0)가 문턱(4.0)보다 **낮아서**, 안 붙들면 5 m/s
로 진입해 3.0 을 고른 차가 감속 도중 4.0 을 지나며 2.0 으로 또 내려간다.
회피 한복판에 목표가 바뀌는 게 제일 위험하다.

**끊겼다 다시 켜지면 되쓴다** (`avoid_cruise_regrab_sec`, 0.5 s). 래치만으로는
부족하다. 래치는 구간을 벗어나는 순간 풀리는데, 접근 중(모드는 아직 GLOBAL,
`_obstacle_on` 만 True)에 검출이 한 프레임 깜빡이거나 모드가 잠시 GLOBAL 로
튕기면 거기서 풀린다. 그러면 다음 프레임에 **이미 줄어든 속도로** 다시 고르게
되어 6 m/s 로 진입해 3.0 을 골랐던 차가 3.5 m/s 지점에서 2.0 으로 떨어진다.
같은 장애물 하나를 피하는 중인데 목표가 바뀌는 것이라, 래치를 둔 이유가 그대로
무너진다.

푸는 걸 늦추는 방식은 안 쓴다 — 그건 글로벌 복귀 후 CSV 속도 회복을 늦춘다.
푸는 건 같은 프레임에 즉시 하고, 이 시간 안에 다시 켜지면 직전 값을 되쓴다.
창이 지나면 새 장애물로 보고 그때 속도로 다시 고른다.

**적용 구간은 `_avoid_speed_capped()`** — 로컬패스 전환 게이트(`_obstacle_on`,
아래 인지거리 표)에서 걸려서 AVOID·REJOIN 을 지나 유지되고, 글로벌패스로
돌아가면 래치가 풀려 **CSV 속도로 복귀한다.** 전환 시점부터 걸리므로 장애물
앞에 도착했을 때는 이미 목표속도다. 여유는 전 구간 충분하다
(`test_avoid_preempt_decel.py`).

| 진입 v | 목표 | 로컬패스 전환 | 감속거리 | 여유 | 감속시간 |
|---|---|---|---|---|---|
| 3.0 | 2.0 | 4.8 m | 0.8 m | +3.9 m | 0.33 s |
| 4.0 | 2.0 | 6.4 m | 2.0 m | +4.4 m | 0.67 s |
| 4.1 | 3.0 | 6.8 m | 1.3 m | +5.5 m | 0.37 s |
| 6.0 | 3.0 | 12.0 m | 4.5 m | +7.5 m | 1.00 s |
| 8.0 | 3.0 | 12.0 m | 9.2 m | +2.8 m | 1.67 s |

문턱 바로 아래(3.9 m/s)가 목표를 한 단 낮게 잡는 만큼 제일 빡빡한데도
4.3 m 남는다. 1~8 m/s 전 구간 최소 여유는 **+1.96 m** 다.

기울기는 `_slew_limit_speed` 가 `avoid_a_brake_mps2`(3.0)로 묶는다.

**그냥 지나가도 되는 장애물은 무시한다.**
`corridor_max_lateral_from_raceline_m` = `HALF_WIDTH_M + 0.03` = **0.18**.
판정은 `_outside_corridor` 에서 `(레이스라인까지 거리 − 장애물 반경) > 기준`
이면 무시하는 식이라, 이 값은 **장애물의 가까운 쪽 끝**이 라인에서 얼마나
떨어져야 없는 셈 치느냐다.

반폭이 0.15 이므로 라인을 그대로 타면 안 닿는다. 여유 3 cm 는 슬립이나
추종 오차로 라인에서 조금 밀린 채 지나갈 경우 몫이다. 이 밖의 물체는
**회피도 안 하고 감속도 안 한다** — `_speed_static_obs` 가 이 필터를 거친
목록이라 두 가지가 한 번에 걸린다. 그냥 CSV 속도로 글로벌 패스를 탄다.

거쳐 온 값: 0.40 → 0.25 → 0.18. 0.40 은 반폭보다 25 cm 넉넉해서 피할 이유가
없는 물체까지 잡았고, 라인을 벗어나는 것 자체가 위험했다.

### 회피는 전부 FGM — 고속에서는 시야를 좁힌다

`avoid_path_mode` 기본값을 `offset` → `straight` 로 되돌렸다. 횡오프셋 기동은
고속 전용으로 만들었지만 실주행에서 계획이 자주 실패했고, 실패하면 어차피
감속 후 FGM 이 받았다 — 두 단계를 거치느라 반응만 늦었다. `offset_*`
파라미터는 되돌릴 때를 위해 남겨 뒀다.

대신 FGM 을 고속에서 쓸 수 있게 만든다. 문제는 FGM 이 **갭만 보고 각을
고른다**는 것이다. 저속에서 정답인 45~60° 가 고속에서도 그대로 나오는데,
그 각은 두 가지를 한꺼번에 망가뜨린다.

1. 조준각이 그대로 요구 조향이 된다. 타이어가 못 내니 차가 그 방향으로 밀린다.
2. `_avoid_target_speed` 의 maneuver 항이 "그 조향을 낼 수 있는 속도" 로
   답한다. 각이 클수록 값이 작아져서, 실측 **0.1 m/s** 까지 떨어졌다 —
   회피하려다 장애물 앞에서 선다.

그래서 고속에서는 **애초에 낼 수 있는 각만 후보로 둔다** (`_fov_for_speed`).
조준거리 `L = v·target_lead_time_s` 의 점을 향해 도는 데 필요한 횡가속이
순수추종 기준 `a = 2v²·sinψ/L` 이므로 `sinψ ≤ a·L/(2v²)`.

| v (m/s) | FOV 반각 |
|---|---|
| ≤4.0 | ±90° (설정값 그대로) |
| 4.5 | ±50° |
| 5.0 | ±18° |
| 6.0 | ±15° |
| 7.0 | ±13° |

`fov_narrow_speed`(4.0) 아래는 손대지 않는다 — 저속 FGM 은 넓은 각이 있어야
막힌 곳에서 빠져나온다. 문턱에서 각이 튀면 조준도 같이 튀므로
`fov_narrow_blend`(1.0 m/s) 구간에 걸쳐 좁힌다. 섞는 건 sin 이 아니라 **각**
이다 — `asin` 은 sin→1 근처에서 수직이라 sin 을 섞으면 문턱 바로 위에서
각이 튄다. `fov_half_min_deg`(12°) 아래로는 안 좁힌다.

**좁히는 건 스캔이 아니라 조준각이다.** 처음에는 `fov_mask` 를 좁혀 스캔
자체를 잘랐는데, 그러면 갭 탐색이 눈을 잃는다. 넓게 열린 쪽이 시야 밖이면
반대쪽 좁은 갭을 고르고, 속도가 흔들릴 때마다 선택이 뒤집혀 좌우로 방황하다
장애물 앞에 선다. 지금은 갭은 설정된 FOV 로 다 보고, 고른 갭 안에서
**트는 각만** `_clamp_to_cone` 으로 줄인다. 오른쪽 +35°~+50° 가 열려 있으면
4 m/s 에서는 +35° 로 가고, 6 m/s 에서는 같은 갭을 향해 +15° 만 튼다.

#### 갭 히스테리시스는 각도로 기억한다

`_select_gap` 은 직전에 따라가던 갭에 붙어 선택이 튀는 걸 막는다. 그 기억을
**work 배열 인덱스**로 들고 있었는데, work 는 FOV 안 빔만 모은 배열이라
FOV 가 변하면 같은 인덱스가 다른 각도를 가리킨다. 속도별 FOV 를 넣는 순간
이게 깨졌다 — 감속으로 FOV 가 18°→80° 로 열리면 인덱스가 통째로 밀려서,
따라가던 갭 대신 반대쪽이 "가깝다" 고 나온다. 이제 각도로 기억한다.

남은 경우의 바닥으로 플래너에 `avoid_fgm_min_speed`(3.0) 를 뒀다. maneuver
사유의 감속에만 걸리고 장애물 거리 기반 감속(static/dynamic)에는 안 걸린다 —
실제로 못 지나가는 상황은 그쪽과 AEB 몫이다.

되돌리려면 `fov_speed_narrow_enable:=false`, 기동으로 돌아가려면
`avoid_path_mode:=offset`.

### 5.6.4 회피 경로 모양 — `avoid_path_mode`

| 값 | 방식 |
|---|---|
| `offset` (기본) | **횡오프셋 기동** — 속도로 길이를 정하는 quintic. 리드 → 진입 → 유지 → 복귀 |
| `straight` | FGM 목표점까지 직선 + `avoid_forward_step_m × avoid_forward_num_points` 전방 직선 연장 |
| `frenet` | 고정 길이 `d(s)` quintic — 진입 → 유지(apex) → 복귀 |

`offset` 실패 시 반대편으로 한 번 재시도하고, 그래도 안 되면 `straight`(FGM 폴백)
로 내려간다. 즉 FGM 은 이제 **주 경로가 아니라 폴백**이다.

#### 왜 바꿨나

`straight` 는 FGM 목표점까지 그은 직선이다. 목표점이 가까우면 그 직선의 꺾임이
그대로 조향 요구가 되는데, 6 m/s 에서 이건 낼 수 없는 값이었다. 차는 조향을 문
채로 벽으로 갔다. `frenet` 은 길이가 고정(1.2 m 급)이라 같은 문제를 속도와 무관하게
겪었다 — 6 m/s 에서 90 m/s² 를 요구했다.

핵심은 **길이를 속도가 정해야 한다**는 것이다. 같은 0.5 m 를 비키더라도 6 m/s 면
6 m 에 걸쳐 펴야 조향이 1.6° 로 떨어진다.

#### 기동 구조 (`offset_maneuver.py`)

순수 계산 모듈이다. ROS 의존이 없어 단위 테스트로 물리를 직접 고정한다.

```
리드 ──────── 진입 ╲──── 유지 ────╱ 복귀 ────────
(라인 유지)   (S커브)   (오프셋)   (S커브)
```

* **리드** — 장애물이 멀면 그만큼 **레이스라인 위에 더 오래 머문다**. 기동은
  필요한 시점에 시작하지, 장애물을 보자마자 시작하지 않는다.
* **진입/복귀** — 5차 다항식. 양 끝에서 `d' = d'' = 0` 이라 조향이 부드럽게
  들어가고 빠진다. 복귀 예산(`a_lat_exit` 2.0)이 진입(`a_lat_enter` 3.0)보다
  작아서 **복귀가 진입보다 완만하다**.
* **유지** — 차체 길이 + 여유만큼 오프셋을 물고 지나간다.

길이는 횡가속 예산에서 나온다 — `L = sqrt(D2_PEAK · |Δd| · v² / a)`. 여기에 조향
한계에서 나오는 최소 길이(`length_for_steer_limit`)를 함께 걸어, 낼 수 없는 기하는
계획 단계에서 **거부**한다 (거부되면 FGM 폴백 또는 제동).

#### 실측 — 검출 거리별 기동

| v | 장애물 | 리드 | 진입 | 조향 | 감속 |
|---|---|---|---|---|---|
| 6 | 14 m | 6.8 m | 6.3 m | 1.57° | 없음 |
| 6 | 8 m | 0.8 m | 6.3 m | 1.57° | 없음 |
| 6 | 6 m | 0 | 5.1 m | 2.41° | 5.94 m/s |
| 6 | 4 m | 0 | 3.1 m | 6.47° | 3.62 m/s |
| 7 | 8 m | 0.4 m | 6.7 m | 1.41° | 없음 |

제때 보면 **감속이 0** 이다. 늦게 보면 딱 그만큼만 줄인다. 4 m 처럼 정말 늦으면
제대로 줄인다 — 그게 맞다.

#### 커밋 — 한 번 그리면 붙든다

매 주기 현재 위치에서 다시 그리면 영원히 곡선의 가장 가파른 앞부분만 반복해서
탄다. 그래서 기동은 한 번 계획하고 **커밋**한다. 다시 그리는 건 세 경우뿐이다.

* 장애물 집합이 바뀌어 계획이 무효 (`avoid_offset_replan_obstacle_m`)
* 차가 계획에서 크게 벗어남 (`avoid_offset_replan_lateral_m`)
* 기동 완료

진행도는 `_maneuver_ds_cache` 에 **증분 누적**한다. `_delta_s` 로 매번 새로 재면
37 m 짜리 폐곡선에서 기동이 반 바퀴에 가까워질 때 랩어라운드로 음수가 나온다.
같은 이유로 진입/복귀 길이를 트랙 길이의 18% (`_MANEUVER_LEN_TRACK_FRAC`) 로
캡한다.

#### 주요 파라미터

| 이름 | 기본 | 뜻 |
|---|---|---|
| `avoid_offset_margin_m` | 속도 정책에서 유도 | 장애물과의 횡 여유 |
| `avoid_offset_a_lat_enter` | 3.0 | 진입 횡가속 예산 [m/s²] |
| `avoid_offset_a_lat_exit` | 2.0 | 복귀 횡가속 예산 — 진입보다 작게 |
| `avoid_offset_a_lat_hard` | 4.5 | 늦게 봤을 때 허용하는 상한 |
| `avoid_offset_enter_max_m` | 9.0 | 트랙 길이로 다시 캡됨 |
| `avoid_offset_exit_max_m` | 12.0 | 〃 |
| `avoid_offset_max_m` | 0.70 | 횡오프셋 **상한**. 실제 값은 벽 예산이 정한다 |
| `avoid_offset_plan_v_floor_mps` | 2.0 | 기동이 트랙에 안 들어갈 때 낮춰 볼 하한 속도 |
| `avoid_offset_plan_v_step_mps` | 0.5 | 그 탐색 간격 |
| `avoid_offset_replan_lateral_m` | — | 이만큼 벗어나면 재계획 |

여유(`margin`)는 속도 정책의 `lateral_margin_m + pass_clear_extra_m` 에서 **유도**한다.
따로 두면 "계획은 비켰는데 속도 정책은 안 비켰다고 보고 제동" 하는 엇박이 난다.

#### 벽 예산 — 계획기가 트랙 경계를 본다

처음에는 계획기가 장애물만 보고 트랙 경계는 경로 충돌검사에 맡겼다. **실차에서
회피하던 방향 벽에 박았다.** 두 군데서 샜다.

1. `avoid_offset_max_m` 을 0.70 으로 박아 뒀는데, 실측 레이스라인→벽 여유는
   중앙값 0.70 m / **최소 0.30 m** 다. 0.70 은 구간의 76% 에서 낼 수 없는 값이다.
   (코드 주석이 근거로 삼던 "최소 0.70 / 중앙값 1.50" 은 센터라인의 **전체 폭**
   이었다. 레이스라인의 한쪽 여유가 아니다.)
2. 방향 선택이 "덜 움직이는 쪽" 이었다. 레이스라인은 코너 안쪽에 붙으므로
   덜 움직이는 쪽이 곧 벽인 경우가 많다.

핵심은 **좌우가 다르다**는 것이다. 실측 좌 중앙값 0.88 m / 우 0.78 m 인데,
둘 중 좋은 쪽만 보면 최소가 0.58 m 로 올라간다 — 방향만 제대로 고르면 이 트랙
어디서든 비킬 수 있다. 그래서 예산을 하나가 아니라 **좌/우 두 개**로 만든다.

`/map` 이 오면 `_build_wall_budget()` 이 레이스라인 각 점에서 법선 방향으로
0.025 m 씩 나가 보며 좌/우 예산을 깐다. 판정은 경로 충돌검사와 **같은 팽창맵**
이라 "계획기는 된다는데 검사기는 아니라는" 상태가 안 생긴다.

계획 시점에는 **오프셋을 물고 있을 구간**(유지 구간)의 최솟값을 넘긴다. 기동
전체로 물으면 16 m 구간의 최솟값이라 트랙에서 제일 좁은 한 곳이 언제나 걸린다
— 실측에서 그렇게 했더니 전 지점에서 계획이 거부됐다. 진입·복귀 램프는 계획이
나온 뒤 `_maneuver_fits_walls()` 가 d(s) 를 점마다 다시 본다.

#### 트랙에 안 들어가면 속도를 낮춘다

진입·복귀 길이는 `sqrt(D2·|Δd|·v²/a)` 라 **v 에 선형**이다. 6 m/s 에서 0.58 m 를
비키려면 기동 전체가 16 m 인데, 우리 트랙은 37 m 다. 반 바퀴짜리 기동이 코너를
몇 개씩 지나가니 벽에 걸릴 수밖에 없다.

그래서 `_plan_fitting_the_track()` 이 속도를 `avoid_offset_plan_v_step_mps` 씩
낮춰 가며 다시 뽑는다. 절반 속도면 절반 길이다. 방향 바꾸기를 **먼저** 시도하므로,
반대쪽이 열려 있으면 감속하지 않는다.

`scripts/check_offset_budget.py` 로 전 지점(748 개)에 정면 장애물을 놓고 잰 결과:

| | 6 m/s 고정 | 속도 탐색 |
|---|---|---|
| 풀속도 성공 | 39% | 39% |
| 감속 후 성공 | — | 37% (목표속도 중앙값 3.5 m/s) |
| 계획 거부 → 감속/TRAILING | 10% | 10% |
| **FGM 폴백** | **51%** | **14%** |
| 벽으로 나간 계획 | 0 | 0 |

FGM 폴백이 곧 벽으로 가는 길이었으므로 이 차이가 크다. 감속은 싫지만, 감속이
필요한 자리는 **애초에 기동이 트랙에 안 들어가는** 자리다. 거기서 대안은 FGM
으로 박거나 줄이거나 둘 중 하나다.

#### 계획 경로는 자르지 않는다

FGM 경로는 조준점까지의 직선이라 앞부분만 살려도 의미가 있다. 그래서
`_truncate_path_at_collision` 은 막힌 데서 자르고 "남은 길이가 충분하면 쓸
만하다" 고 답한다.

기동에는 그 규칙을 쓰면 안 된다. 진입-유지-복귀가 한 덩어리라 벽에 걸려 잘려
나가는 건 언제나 뒷부분이다. 그걸 받으면 차는 **진입만 타고 최대 오프셋에
도달하는데, 그 지점이 바로 잘려 나간 벽**이다. 그래서 계획 기동은
`_path_fully_clear()` 로 통째로 검사하고, 한 점이라도 걸리면 반대편 → FGM
폴백 순으로 넘긴다.

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

> **1번 항목은 `offset` 기동이 없앤 한계다.** 아래 표는 FGM 목표점까지 직선을
> 긋던 시절 이야기고, 지금은 폴백 경로에만 해당한다. 계획 기동은 목표점 거리가
> 아니라 **횡가속 예산**이 길이를 정하므로, 7 m/s 에서도 6.7 m 에 걸쳐 펴서
> 1.41° 로 지나간다 ([5.6.4](#564-회피-경로-모양--avoid_path_mode)). 2번(정지
> 한계)은 여전히 유효하다 — 늦게 보면 계획도 속도 상한을 건다.

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
| `adaptive_min_points` | **`True`** | 원거리에서 최소 점수를 낮춤 | static / integrated |
| `consistent_centroid` | `False` | 추적은 centroid, 반지름은 분위수 | static / integrated |
| `tracker_mode` | `ema` | `kf` — 등속 칼만 | integrated |
| `wall_residual_guard` | **`True`** | 팽창 벽에 붙은 잔차에 높은 기준 | static / integrated |
| `bubble_speed_scale_enable` | `False` | FGM 버블을 속도에 비례 | fgm |

**적응형 임계(`cluster_mode: adaptive`).** 끊는 기준을
`d_max(r) = r·sin(Δφ)/sin(λ−Δφ) + 3σ` 로 잡고 `[0.05, 0.35] m` 로 클램프한다.
`λ`(기본 10°)는 "이보다 비스듬한 면은 같은 물체로 안 본다"는 허용 입사각이다.

여기서 알아둘 것: 고정 `0.28 m` 는 사실상 **8 m 용으로 튜닝된 값**이다. λ=10°
에서 두 곡선은 8 m 부근에서 만난다. 즉 적응형으로 바꾸면 원거리가 아니라
**근거리가 크게 엄격해진다** (2 m 에서 0.115 m). 가까운 물체를 잘 분리하는 게
이득이지만, 반대로 **가까운 상대차 하나가 2~3 조각으로 쪼개질 위험**이 있다.
조각 각각이 `min_obstacle_size_m`(0.12) 미만이면 전부 버려져서 **장애물이 통째로
사라진다.** 실차에서 이 모드를 켤 때는 상대차를 2 m 앞에 세워두고
`/static_obstacles` 개수가 1 로 유지되는지부터 봐야 한다. 쪼개지면
`abd_lambda_deg` 를 15~20° 로 올린다 (임계가 커진다).

### 작은 장애물을 얼마나 멀리서 보는가 — 게이트 두 개를 같이 봐야 한다

대회 장애물은 최대 50×50 cm 라 그보다 작은 것도 봐야 하는데, 검출은 게이트
두 개를 연달아 통과해야 하고 **둘 다 거리에 비례해 나빠진다.**

1. **점 수.** 폭 `w` 물체가 거리 `r` 에서 남기는 점은 `w/(r·Δφ)` 개다.
   실측 `Δφ = 0.00421 rad`(0.241°, Slamtec T1) 이라 고정 10 점 기준의 한계
   거리는 `w × 23.8 m` 다.
2. **span.** 측정 span 은 물체 폭이 아니라 **양 끝 '맞은' 빔 사이 거리**다.
   실제 폭보다 짧고, 얼마나 짧은지는 빔 격자와 물체의 위상에 달렸다.
   11 m 에서 빔 간격이 4.6 cm 라 이 손실이 `min_obstacle_size_m` 앞에서 그대로
   문제가 된다.

한쪽만 풀면 다른 쪽이 그대로 잘라 낸다. 그래서 `adaptive_min_points`(→ `True`)
와 `min_obstacle_size_m`(0.14 → **0.12**)을 같이 잡았다. 위상을 8 개로 훑어
**모든 위상에서** 검출되는 최대 거리 (`max_obstacle_range_m` 11 m 에서 끊음):

| 물체 폭 | 이전 (고정 10점, 0.14) | `adaptive` 만 (0.14) | 지금 (둘 다) |
| --- | --- | --- | --- |
| 15 cm | 1.6 m | 1.6 m | **5.9 m** |
| 20 cm | 4.7 m | 9.5 m | **11.0 m** |
| 25 cm | 5.9 m | 11.0 m | **11.0 m** |
| 30 cm | 7.1 m | 11.0 m | **11.0 m** |
| 50 cm | 11.0 m | 11.0 m | 11.0 m |

50 cm 는 원래도 사거리 끝까지 보였다. 문제는 그 아래였다 — 20 cm 가 4.7 m
에서야 보이면 6 m/s 에서 충돌 0.8 초 전이라, 회피 게이트를 12 m 로 열어 놔도
회피가 아니라 AEB 가 된다.

**위상을 훑는 이유.** 차가 다가가는 동안 격자 위상이 프레임마다 바뀐다. 운
좋은 위상 하나만 재면 한계가 크게 낙관적으로 나온다 — 15 cm 를 1.6 m 대신
7.1 m 로 봤다. 검출이 M-of-N(6 중 4)을 통과하려면 특정 위상이 아니라 위상
전반에서 떠야 한다.

**대가와 안전장치.** 원거리 문턱이 `min_cluster_points_floor`(3)까지 내려가고
span 게이트도 2 cm 낮아졌으니 유령 장애물이 늘 여지가 있다. 막는 건 세 겹이다:
`wall_residual_guard`(기본 켬, 벽 0.20 m 안쪽 잔차에는 14점·span 0.20 요구),
M-of-N 확정, 그리고 span 게이트 자체다. 4 cm·8 cm 짜리 반사는 어떤 위상에서도
안 뜨는 걸 확인했다.

되돌릴 때는 **둘 다** 되돌려야 한다. `min_obstacle_size_m` 만 0.14 로 올리면
20 cm 는 여전히 9.5 m 라 멀쩡해 보이는데 15 cm 만 조용히 1.6 m 로 죽는다.
근거와 표는 `test/test_small_obstacle_detection.py` 가 배포 CFG 로 고정한다.

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
밴드를 벗어나 "장애물"로 샌다. 켜면 팽창 경계에서 `wall_clearance_m`(0.35) 안쪽에
있는 클러스터에는 `near_wall_point_gate()` + `near_wall_min_span_m`(0.20) 를
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
- `/planner/local_path_planned` ← `local_planner_node` (Bool). 지금 주는
  `/local_path` 가 계획된 기하인지 — Stanley 의 곡률 FF 스위치
  ([위](#계획-경로에서-ff-를-켜야-한다-plannerlocal_path_planned))

코드 수정 후: `colcon build --packages-select path_following`

---

## 12. CPU 최적화 (20260822)

주행이 안정된 뒤 CPU 를 줄였다. **거동은 그대로 두고 낭비만** 걷어내는 게
조건이라, 고치기 전에 재고 고친 뒤 다시 쟀다. 측정 도구는 `debug/` 에 있다.

```bash
python3 debug/bench_hotpaths.py --json debug/bench_before.json   # 기준선 저장
python3 debug/bench_hotpaths.py --base debug/bench_before.json   # 전후 비교
python3 debug/bench_msgbuild.py     # ROS 메시지 조립 비용
python3 debug/cpu_by_node.py 15     # 런치가 떠 있을 때 노드별 실측
```

### 무엇이 CPU 를 쓰고 있었나

짐작과 실측이 갈렸다. 기하 계산이 문제일 거라 보고 들어갔는데, **가장 큰
항목은 ROS 메시지 조립**이었다. `PoseStamped()` 하나가 18 µs 라 140점짜리
`Path` 를 만드는 데 2.6 ms 가 든다. 33 Hz 면 그것만으로 코어의 9 % 다.

| 자리 | 전 | 후 | 배속 |
|------|----|----|------|
| `Path` 140점 (Stanley `tracked_path`) | 2615 µs | 148 µs | **18x** |
| `Path` 750점 (`/raceline_csv_path`) | 14090 µs | 27 µs | **98x** |
| `MarkerArray` (장애물 시각화) | 70 µs/개 | 재사용 | — |
| `lateral_distance_to_closed_polyline` | 549 µs | 56 µs | **9.8x** |
| `closest_projection_on_loop` (Stanley) | 359 µs | 64 µs | **5.6x** |
| `sliding_xy` | 377 µs | 89 µs | **4.2x** |
| `_corridor_clear_distance` (FGM) | 58 µs | 25 µs | **2.4x** |
| `_closest_on_loop` (플래너 Frenet) | 75 µs | 56 µs | **1.3x** |

기하 함수 합계로 코어 **22.5 % → 9.1 %**, 여기에 시각화 메시지에서 **12 %p**
가량이 더 빠진다.

### 고친 방식

**1. 안 바뀌는 걸 다시 만들지 않는다.**
`lateral_distance_to_closed_polyline` 은 호출마다 750점 리스트를 ndarray 로
바꾸고 `roll` 두 번에 곱셈까지 다시 했다. 그 값들은 레이스라인에서만 정해진다.
`track_sliding.segment_geometry` 가 리스트 **신원** 으로 캐시한다 (`id` 만
키로 쓰면 리스트 해제 후 주소 재사용 때 남의 기하를 준다 — 원본을 같이 들고
있는 이유다). `_closest_on_loop` 도 같은 이유로 `_abx_np/_ab2_np` 를 CSV 를
읽을 때 한 번만 만든다.

**2. 파이썬 루프를 numpy 한 번으로.**
`closest_projection_on_loop` 은 앵커폭 120 이면 세그먼트 241 개를 파이썬으로
돌았다. 인덱스 순서를 **예전 루프와 똑같이** 만들어 `argmin` 에 넘긴다 —
`d2 < best_d2` 도 `argmin` 도 동점이면 먼저 것을 남기므로, 순서가 같으면 고르는
세그먼트도 같다. 여기서 나온 인덱스가 다음 주기 앵커라 하나만 어긋나도 궤적이
갈린다.

**3. 각도마다 하던 삼각함수를 스캔당 한 번으로.**
FGM 코리도 검사는 스캔당 수십 번 도는데 (갭 적합성 × 갭 수 + 조준 후보 11개),
매번 `arctan2(sin, cos)` 래핑에 `cos`·`sin` 을 **전 빔** 에 걸었다. 회전 대상은
같은 스캔이므로 직교좌표를 한 번 펴 두면 각도별로 남는 건 스칼라 회전뿐이다.

**4. 한 주기에 두 번 하던 걸 한 번으로.**
장애물 필터 네 종류가 게이트 판정에서 한 번, `_obstacles_remain` 에서 또 한 번
불렸다. 입력·파라미터·TF 가 그 사이에 안 바뀌므로 두 번째는 같은 답을 다시 만드는
것뿐이다. `_filter_cached` 가 `_tf_cycle_id` 로 주기마다 비운다.

**5. 메시지 객체를 돌려쓴다.**
`publish()` 는 그 자리에서 직렬화하고 돌아오므로, 반환 뒤에는 미들웨어가 파이썬
객체를 안 들고 있다. 노드가 각자 프로세스라 intra-process 공유도 없다. 그래서
`tracked_path` 와 장애물 마커는 풀에서 꺼내 값만 갈아 끼운다. `/raceline_csv_path`
는 **내용이 아예 안 변하므로** 한 번 만들고 스탬프만 간다.

### 일부러 안 건드린 것

**`/local_path` 조립 (AVOID 중 3.3 ms/주기, 13 %).** 여기도 풀을 쓰면 18배가
나오지만, 플래너는 `_last_good_avoid_path` 와 `_rejoin_path_msg` 를 **주기를
넘겨 들고 있다가 다시 발행한다.** 포즈를 돌려쓰면 다음 주기 계획이 그 저장본을
조용히 덮어써서, "막혔을 때 붙들던 마지막 정상 경로" 가 방금 기각된 경로가 된다.
버퍼를 번갈아 쓰면 풀리지만 `avoid_hold_max_sec` 만큼의 주기를 살려 둬야 해서
버퍼 수가 그 설정에 묶인다. 이득이 회피 중에만 생기는 것에 비해 틀렸을 때
대가가 커서 남겨 뒀다.

**`cluster_scan_xy` (~1.2 %).** 세그먼트별 `np.mean`/`argmin` 을 `reduceat` 로
접으면 빨라지지만 합산 순서가 달라져 클러스터 중심이 마지막 자리에서 흔들린다.
탐지 수치는 지금 겨우 맞춰 놓은 자리라, 이만한 이득에 건드릴 값이 아니다.

**AEB 50 Hz 재평가.** 스캔은 40 Hz 라 열 번 중 두 번은 안 바뀐 스캔을 다시 본다.
다만 속도는 그 사이에도 바뀌므로 통째로 건너뛸 수 없고, 안전 노드라 손대지 않았다.

### 같은 답이 나오는지 어떻게 확인했나

`test/test_hotpath_equivalence.py` 가 예전 구현을 그대로 박아 두고 같은 입력으로
비교한다. 트랙 근처·트랙 밖·먼 점을 섞고, 투영은 **앵커가 상태로 이어지므로**
한 바퀴를 순서대로 따라가며 전 구간을 본다.

기준은 자리마다 다르다.

- **비트 단위 동일**: 투영·Frenet 처럼 결과가 다음 주기 상태로 남는 것. 1 ulp 가
  누적되면 궤적이 갈린다.
- **부동소수 잡음 이내 + 문턱 판정 동일**: FGM 코리도 거리처럼 한 번 재서 비교하고
  버리는 값. 스칼라 회전은 반올림 순서가 달라 ≤1 ulp 차가 나는데, 입력이 ±1 cm
  LiDAR 잡음인 자리에서 1e-16 을 지킬 이유가 없다. 대신 문턱 비교 결과가 같은지를
  따로 묶었다.

캐시·풀 자체의 함정도 시험으로 막았다 — 주기가 넘어갔는데 재사용하기
(`test_filter_cache.py`), 경로가 짧아졌는데 옛 꼬리 남기기
(`test_viz_path_reuse.py`), 마커 슬롯 겹치기 (`test_marker_pool.py`).

---

## 13. CPU 최적화 2차 — 아무도 안 듣는 발행 지우기 (20260822)

1차(12장)는 우리 코드의 계산과 메시지 **조립** 을 줄였다. 그 뒤 실차에서 노드별로
다시 재 보니 숫자가 여전히 컸고, 이번엔 원인이 다른 데 있었다.

```bash
python3 debug/cpu_by_node.py 20          # 노드별 (런치가 떠 있어야 한다)
python3 debug/cpu_by_thread.py local_planner 12   # 노드 안에서 스레드별
python3 debug/bench_executor.py full     # 콜백을 비운 노드의 rclpy 오버헤드
python3 debug/topic_consumers.py         # 토픽을 누가 듣는지
python3 debug/bench_path_publish.py      # 큰 Path 를 내보내는 비용
python3 debug/check_sub_count.py         # 구독자 수 조회를 믿어도 되나
```

### 실측이 말한 것

차를 **세워 둔 채** (회피도 없고 장애물도 없다) 잰 값이다.

| 노드 | CPU% | 메인스레드(파이썬) | 그 외(DDS) |
|------|------|--------------------|------------|
| `local_planner_node` | 41.9 | 41.0 | 2.5 |
| `stanley_waypoint_follow_node` | 39.6 | 39.5 | 2.2 |
| `emergency_brake_node` | 31.5 | 30.9 | 2.2 |
| `integrated_obstacle_node` | 27.0 | — | — |
| `fgm_node` | 20.1 | — | — |

스레드가 노드당 13개라 DDS 전송이 범인일 줄 알았는데 **94 % 가 메인스레드**,
즉 파이썬이었다. 그런데 12장에서 최적화한 함수들을 다 더해도 41 % 가 안 나온다.

그래서 **콜백을 전부 빈 함수로 둔 노드** 를 같은 구독 구성으로 띄워 봤다
(`bench_executor.py`). 하는 일이 아무것도 없는데 **23.1 %** 였다.

| 구성 | CPU% | 차이가 말하는 것 |
|------|------|------------------|
| 구독 6 + TF + 퍼블리셔 10 + 타이머 2 | 23.1 | — |
| TF 리스너 뺌 | 14.4 | TF 리스너 하나가 **8.7 %p** |
| 퍼블리셔 10개 뺌 | 19.8 | 퍼블리셔는 **가만히 있어도** 개당 0.33 %p |
| 타이머만 | 3.2 | 구독 6개가 11 %p |

rclpy 실행기는 깨어날 때마다 대기셋을 처음부터 다시 짠다. `/tf` 가 127 Hz 라
초당 400 번 가까이 깨우고, 그때마다 엔티티 전부를 다시 등록한다. 즉 **남은
비용은 계산이 아니라 엔티티 수와 깨우는 횟수** 였다.

### 진짜 낭비: 구독자가 0 이어도 직렬화를 다 한다

`topic_consumers.py` 로 소비자를 훑으니 path_following 이 내는 24개 중 **13개는
Foxglove 전용** 이었다. 그리고 rclpy 는 듣는 데가 없어도 파이썬 메시지를 C 로
옮기는 일을 끝까지 한다.

| | 구독자 0 일 때 발행 비용 |
|---|---|
| `Float64` | 11 µs |
| `Path` 140점 (`/waypoint_tracked_path`) | 0.9 ~ 1.7 ms |
| `Path` 750점 (`/raceline_csv_path`) | 5.6 ~ 6.2 ms |
| `get_subscription_count()` | **1.8 µs** |

레이스 중에는 Foxglove 를 닫아 두는데, 그동안 Stanley 는 33 Hz 로 아무도 안 보는
140점 경로를 계속 직렬화하고 있었다. 검사 한 번이 발행의 1/700 이라 물어보고
거르는 쪽이 언제나 싸다.

### 고친 방식

`path_following/viz_gate.py` 의 `has_listener(pub)` 하나로, **시각화 전용 토픽만**
"듣는 데가 있을 때만" 내보낸다.

| 노드 | 게이트 건 토픽 |
|------|----------------|
| `stanley_waypoint_follow_node` | `/waypoint_tracked_path`, `/stanley/debug`, `/control/*` 5개 |
| `local_planner_node` | `/raceline_csv_path`, `/planner/speed_condition`, `/planner/speed_reason` |
| `emergency_brake_node` | `/emergency_brake/ttc` |
| `fgm_node` | `/fgm_gap_marker`, `/fgm_gap_markers`, `/fgm_debug_scan` |
| `integrated_obstacle_node` | `/visualization_marker_array` |

**제어 경로에는 걸지 않았다.** `/drive`, `/local_path`, `/emergency_brake`,
`/planner/speed_scale`, `/planner/mode`, `/planner_path_override_active`,
`/planner/local_path_planned`, `/planner/fgm_enable`, `/planner/fgm_prefer_angle`,
`/fgm_target`, `/aeb/escape_reverse` 는 그대로다. 구독자가 붙기 직전에 한 장을
건너뛰면 상태를 나르는 토픽은 거동이 달라진다.
`test_viz_gate.py::test_the_control_path_topics_have_no_gate` 가 소스를 읽어
이걸 강제한다 — 나중에 누가 게이트를 넓히면 시험이 먼저 깨진다.

Foxglove 패널을 열면 25 ms 만에 다시 나가고, 닫으면 27 ms 만에 멎는다
(`check_sub_count.py`). 33 Hz 기준 한 프레임이라 화면에서는 티가 안 난다.
`get_subscription_count()` 는 퍼블리셔 **자신이 매칭한 상대** 를 세는 값이라,
외부 노드에서 `get_subscriptions_info_by_topic` 을 부를 때 같은 디스커버리
지연을 타지 않는다.

### 이번에도 안 건드린 것

**TF 리스너 (노드당 8.7 %p, 4개 노드면 약 35 %).** `/tf` 가 127 Hz 로 오는데
소비자는 33~40 Hz 로만 조회한다. 발행 쪽 주기를 낮추면 네 노드가 같이 싸지지만,
그건 측위·제어가 보는 자세의 신선도를 깎는 일이다. 6 m/s 에서 10 ms 는 6 cm 라
"주행에 영향 없음" 조건을 못 지킨다.

**퍼블리셔 존재 비용 (개당 0.33 %p).** 시각화 퍼블리셔 13개를 아예 안 만들면
4 %p 가 더 빠진다. 파라미터로 끄게 할 수는 있지만, 그러면 Foxglove 를 켜려고
런치를 다시 띄워야 한다. 게이트는 실행 중에 알아서 붙었다 떨어지므로 그쪽을 골랐다.

---

## 10. 튜닝 요약 (현재 CFG 기본값)

| 노드 | 주요 파라미터 |
|------|----------------|
| `stanley_waypoint_follow_node` | `max_steering_angle` ±40°, `stanley_k` 1.5, `max_drive_speed` 1.5 |
| `local_planner_node` | `avoid_on_m` 1.8, `avoid_pass_rear_x_m` -0.35, 직진 leg 30×0.15 m, `path_check_inflation_m` 0.25, `avoid_a_lat_mps2` 4.0 |
| `drive_strategy_node` | 직선 `speed_straight_mul` 2.0, 곡선 `speed_curve_mul` 0.5 |
| `fgm_node` | 목표점 = `target_lead_time_s` 0.70 × 속도 (1.0~5.0 m), `scan_max_range_m` 10.0, `bubble_radius_m` 0.20, `gap_edge_inset_deg` 3, `corridor_half_width_m` 0.22 |
| `static_obstacle_node` | `max_obstacle_size_m` 0.85, `min_obstacle_size_m` 0.12, `adaptive_min_points` true (floor 3, `min_arc_m` 0.07), `max_obstacle_range_m` 11.0 |
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

#### Foxglove 에서 `laser` 프레임이 통째로 안 보일 때

증상: `/local_path`(map 프레임)는 보이는데 `/scan` 과 FGM 갭 마커는 안 보이고,
`Missing transform from frame <laser> to frame <map>` 이 뜬다. 3D 패널의 프레임
목록에 `map`/`odom`/`base_link` 만 있고 `laser` 가 없다.

원인은 `/tf_static` QoS 다.

| | Reliability | Durability |
|---|---|---|
| `sensor_static_tf` (발행) | RELIABLE | **TRANSIENT_LOCAL** |
| ROS 노드들 (tf2 리스너) | RELIABLE | TRANSIENT_LOCAL |
| `foxglove_bridge` (구독) | BEST_EFFORT | **VOLATILE** |

`/tf_static` 은 노드가 뜰 때 딱 한 번 나간다. TRANSIENT_LOCAL 구독자는 늦게
붙어도 그 latch 를 받지만, **VOLATILE 구독자는 못 받는다.** 그래서 브릿지가
`sensor_static_tf` 보다 늦게 뜨면 `base_link→laser` 를 영영 못 받는다. 다른
ROS 노드는 다 멀쩡하니 젯슨 쪽 로그로는 안 잡힌다. 배터리 갈고 노드를 다시
켤 때마다 됐다 안 됐다 한 게 이 시작 순서 문제였다.

`sensor_static_tf` 가 1 Hz 로 다시 발행한다. 값이 매번 같으니 tf2 는 덮어쓰기라
무해하고, 늦게 붙는 구독자만 구제된다.

이미 떠 있는 상태에서 즉시 살리려면 (재시작 불필요, 브릿지는 이미 구독 중):

```bash
timeout 3 ros2 run tf2_ros static_transform_publisher \
  --x 0.31 --y 0.0 --z 0.20 --qx 0 --qy 0 --qz 0 --qw 1 \
  --frame-id base_link --child-frame-id laser
```

값은 `sensor_static_tf.cpp` 와 같아야 한다. 다르면 TF 가 어긋난다.

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


git pushall