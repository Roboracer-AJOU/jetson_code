# 명령어

cd ~/f1tenth_ajou
source install/setup.bash

ros2 launch localization_layer cartographer_mapping_launch.py

ros2 launch localization_layer cartographer_localization_launch.py

ros2 launch localization_layer amcl_localization_launch.py

ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765

ros2 launch path_following path_follow_static_dynamic_avoid_launch.py


source /opt/ros/humble/setup.bash
source /home/nvidia/f1tenth_ajou/install/setup.bash

ros2 run path_following control_node



# localization_layer 수정 기록

Cartographer 로컬라이제이션 튜닝하면서 뭘 왜 바꿨는지 여기에 순서대로 남겨둡니다.
나중에 왜 이 값인지 헷갈리지 않으려고 적어둡니다.

---

## 수정 1 — 2026-08-03

**증상**: 자율주행 중 위치가 틀어진 뒤 원래대로 못 돌아옴. Foxglove에서 base_link 주변으로 여러 색 선이 부채꼴로 퍼지는 게 보임 (pose graph constraint 시각화로 추정).

**왜 이런 일이 생기는지**: 사실 애매하긴 한데 일단 고쳐봤어요
```
넓은 탐색범위(3.5m) + 많이 검토(sampling 0.2) + 낮은 통과기준(0.55)
    → 매칭 후보를 아주 많이, 아주 넓게, 아주 관대하게 받아들임
    → 트랙이 원형 루프라서 여러 구간이 라이다한테 비슷하게 보임
    → "괜찮아 보이는" 매칭 중 상당수가 사실은 틀린 곳(비슷하게 생긴 다른 구간)
    → 틀린 매칭이 포즈그래프에 제약으로 들어가서, 최적화할 때 위치가 엉뚱하게 끌려감
    → 부채꼴로 여러 후보선이 퍼지고, 틀어진 뒤 복구가 안 됨
```

**수정**: `src/localization_layer/config/cartographer_2d_localization.lua`

| 파라미터 | 이전 | 이후 | 뭘 하는 값인지 |
|---|---|---|---|
| `constraint_builder.sampling_ratio` | 0.2 | 0.1 | 매칭 후보를 몇 %나 검토할지. 낮출수록 검토량↓ → 틀린 것도 덜 섞임 |
| `constraint_builder.min_score` | 0.55 | 0.62 | 매칭 통과 기준 점수(0~1). 제일 직접적인 손잡이. 올릴수록 확실한 매칭만 통과 |
| `fast_correlative_scan_matcher.linear_search_window` | 3.5 | 2.2 | 몇 m 반경까지 찾아볼지. 좁힐수록 안쪽/바깥쪽 루프를 혼동할 위험↓ |

**방향**: 넓고 느슨하게 찾기보다, 좁고 확실한 매칭만 받아들이는 쪽으로 조임.

**확인 필요**: 이 값으로 주행했을 때 (1) 부채꼴 패턴이 줄어드는지, (2) 그래도 여전히
"틀어진 뒤 복구 안 됨" 현상이 남아있는지 Foxglove로 확인 필요.

**되돌리는 법**: `git diff HEAD -- src/localization_layer/config/cartographer_2d_localization.lua`
로 지금까지 바뀐 것 전체 확인 가능. 이 수정만 되돌리려면 위 표의 "이전" 값으로 다시 바꾸면 됨.

---

## 수정 2 — 2026-08-03

**배경**: 그동안 휠오돔(`vesc_wheel_odom.py`)의 회전(yaw)은 "조향각(서보각)을 bicycle model에
넣어서 추정"하는 방식이었음. 근데 이건 서보각 오프셋/스케일 오차가 그대로 yaw 오차로 전달되는
문제가 있었음 (예전 맵 깨짐 사고도 이 계산 관련). IMU가 새로 붙어서 이제 각속도를 실측할 수
있게 됨 → 팀원 제안대로 **조향각 기반 추정을 걷어내고, IMU 자이로 실측값을 그대로 yaw rate로
사용**하도록 변경.

**수정**: `src/localization_layer/scripts/vesc_wheel_odom.py`,
`src/localization_layer/launch/localization_launch_common.py`

- 조향각 관련 파라미터/구독/계산 전부 제거 (`servo_feedback_deg_topic`, `max_steer_rad`,
  `steer_scale`, bicycle model 계산 등 — 이제 안 씀)
- `/imu/data` 구독 추가, `angular_velocity`를 yaw rate로 직접 사용
- 새 파라미터 `imu_yaw_axis` (기본값 `'z'`) — gz/gy 중 어느 축이 실제 yaw인지 아직 실측
  미확정이라, 확인되면 launch 파일에서 이 값만 `'y'`로 바꾸면 됨 (코드 재수정 불필요)
- 속도(VESC)는 그대로 사용, 바뀐 건 회전 계산 방식뿐

**확인 필요 (다음에 차 있을 때)**:
1. ~~`ros2 topic echo /imu/data --field angular_velocity` 켜놓고 실제로 좌우 회전시켜서
   `y`/`z` 중 어느 축이 반응하는지 확인~~ → **완료 (아래 결과 참고)**
2. ~~`ros2 launch localization_layer cartographer_localization_launch.py use_ebimu:=true`로
   켜서 `vesc_wheel_odom` 로그에 에러 없이 뜨는지, `/odom` 토픽이 정상 발행되는지 확인~~ →
   **완료.** `ebimu_driver`/`vesc_wheel_odom`/`cartographer_node` 정상 실행, 파라미터
   (`imu_yaw_axis=z`, `imu_topic=/imu/data`) 정상, `/odom` 50Hz로 정상 발행 확인.
3. 실제 주행하면서 코너에서 헤딩이 이전보다 안정적인지 Foxglove로 비교 — **다음에 확인**

### /odom 회전값 반응 확인 (2026-08-03)

`control_node` 켜지지 않은 상태로는 `/vehicle/speed_mps`가 없어서 속도 0 취급 →
`min_speed_for_yaw_mps` 안전장치 때문에 `/odom` 회전값이 계속 0으로 나오는 문제 발견.
`control_node` 실행 후 재확인:

```
/odom twist.twist.angular.z, 8초 샘플(281개), 실주행 중
min=-1.389 max=0.432 절대값최대=1.389 rad/s
```

정상적으로 조향에 반응함 확인. **주의**: 이 노드가 제대로 작동하려면 로컬라이제이션 +
`control_node` 둘 다 켜져 있어야 함 (`control_node` 없으면 속도 0으로 회전값도 안 나옴).

**되돌리는 법**: `git diff HEAD -- src/localization_layer/scripts/vesc_wheel_odom.py
src/localization_layer/launch/localization_launch_common.py`

### yaw 축 확인 결과 (2026-08-03)

실제 주행 중 `/imu/data`의 `angular_velocity` 8초 샘플링(574개) 결과:

| 축 | 절대값 최대 |
|---|---|
| x | 0.339 rad/s |
| y | 0.348 rad/s |
| **z** | **1.541 rad/s** ← 다른 축보다 4~5배 큼 |

**결론: z축이 맞음.** `imu_yaw_axis` 기본값(`'z'`) 그대로 사용, 코드 수정 불필요.

---

## 수정 3 — 2026-08-03

**패키지 위치**: 이 수정은 `localization_layer`가 아니라 `ebimu_pkg`
(`src/ebimu_pkg/ebimu_pkg/ebimu_driver.py`)에 있음. 로컬라이제이션이 IMU
데이터를 직접 쓰기 때문에(수정 2 참고, `vesc_wheel_odom.py`가 `/imu/data`의
각속도를 yaw rate로 사용) 여기에도 같이 남겨둠. 자세한 내용은
`src/ebimu_pkg/README.md` 참고.

**증상**: `ros2 topic hz /imu/data` 확인 결과 평균 100Hz인데 메시지 간격이
0.001s ~ 0.026s로 26배 차이 나게 들쭉날쭉함 (burst 패턴 — 몰려왔다 끊기고
다시 몰림). 실주행 중 커브 구간에서 로컬라이제이션 위치가 튐 (Foxglove에서
base_link 주변에 부채꼴로 라이다 스캔이 흩어져 보임).

**원인**: `ebimu_driver.py`의 100Hz 타이머가 그 순간 시리얼 버퍼에 쌓인
프레임을 있는 대로 전부 publish함. 타이밍이 밀리면 한 틱에 여러 프레임이
쌓이고, 그게 전부 거의 동시 timestamp로 발행됨 → burst. Orientation-only
프레임의 각속도 계산(`(각도차) / (now - last_time)`)에서 `dt`가 burst 때문에
순간적으로 작게 찍히면 각속도가 스파이크로 튐. 직선에서는 각도 변화가
작아 안 보이지만, **커브에서는 실제 각도 변화가 커서 이 오차가 그대로
증폭** → yaw rate가 부정확해짐 → 이 값을 그대로 쓰는 `vesc_wheel_odom.py`
(수정 2) 통해 로컬라이제이션까지 흔들림.

**수정**: 처음엔 "최신 프레임 하나만 publish, 나머지는 버림"으로 고쳤었는데,
이러면 실제 측정값을 버리게 되는 문제가 있어서(타이밍을 깔끔하게 보이려고
정확도를 희생한 것) **프레임을 하나도 안 버리고, 대신 한 틱에 몰린
프레임들에 실제 샘플링 간격만큼 역산한 timestamp를 균등하게 매기는
방식**으로 재수정함. 자세한 코드는 `src/ebimu_pkg/README.md` v2 항목 참고.

**확인 필요**: `colcon build --packages-select ebimu_pkg` 후 재시작해서
`ros2 topic hz /imu/data`로 간격 std dev가 줄었는지, 실주행 커브에서
위치 튐이 줄었는지 확인 필요.

**되돌리는 법**:
```bash
git checkout -- src/ebimu_pkg/ebimu_pkg/ebimu_driver.py
```

---

## 수정 4 — 2026-08-03

**증상**: 실주행 중 속도 2.5m/s까지는 괜찮은데, **3m/s부터 맵(로컬라이제이션 위치)이
확 틀어짐.**

**분석**: 라이다는 `lidar_scan_frequency=20Hz`로 고정돼 있음 (Jetson 실시간성
때문에 일부러 낮춰둔 값, [localization_launch_common.py:71-73](launch/localization_launch_common.py#L71)).
스캔 간격 0.05초 동안은 IMU+오돔 예측만으로 버텨야 하는데, 스캔매칭이
찾아볼 수 있는 오차 허용 범위가 고정값(`real_time_correlative_scan_matcher.
linear_search_window = 0.30`, 약 ±15cm)임. 2.5m/s면 0.05초에 12.5cm 이동,
3m/s면 15cm 이동 — **탐색 범위 한계에 거의 걸치는 수준이라, 예측이 조금만
부정확해도 3m/s부터 탐색 범위를 벗어나 스캔매칭이 엉뚱한 곳에 붙어버림.**

**40Hz로 스캔 주기를 올리는 것도 검토했으나 보류**: 스캔매칭 연산량 2배,
포즈그래프 최적화 빈도도 증가 → 젯슨 CPU가 못 따라가면 오히려 더 늦게
처리된 포즈로 계산하게 돼서 지금 문제가 악화될 위험이 있음. 그래서 **CPU
부담 없는 예측 정확도 개선(오돔 적분)부터 먼저 시도.**

**수정**: `src/localization_layer/scripts/vesc_wheel_odom.py`

- 기존엔 `self._yaw_raw += omega * dt`가 `_on_timer`(50Hz 타이머)에서만
  실행됐음. `_on_imu` 콜백은 최신 각속도 값만 저장해두고, 실제 적분은
  타이머가 그 순간의 최신값 하나로 대신 계산 — **IMU는 100Hz로 들어오는데
  실제로는 50Hz로만 샘플링(zero-order hold)해서 절반은 버려지는 구조**였음.
- **`_on_imu` 콜백 안에서, IMU 메시지 자신의 `header.stamp`(수정 3에서
  burst 보정된 정확한 timestamp) 기준으로 직접 적분**하도록 변경. 이제
  `self._yaw_raw`는 IMU가 오는 즉시(실측 ~100Hz) 갱신됨.
- `_on_timer`(50Hz)는 그대로 저역통과 필터 적용 + x,y 병진 적분 + `/odom`
  publish만 담당 (yaw 적분 라인만 제거).

**기대 효과**: yaw 예측 정밀도가 올라가서, 3m/s에서 스캔 간 예측 오차가
스캔매칭 탐색 범위(±15cm) 안에 들어올 여유가 조금 더 생김.

**확인 필요**: `colcon build --packages-select localization_layer` 후
재시작해서, 3m/s로 재주행 테스트 — 여전히 틀어지면 다음 후보는
`real_time_correlative_scan_matcher`의 탐색 범위를 넓히거나(CPU 부담 적음),
그래도 안 되면 `lidar_scan_frequency`를 조금씩(20→25 등) 올리며
`tegrastats`로 CPU 확인.

**되돌리는 법**:
```bash
git diff HEAD -- src/localization_layer/scripts/vesc_wheel_odom.py
```
(`_on_imu`에서 timestamp 기반 적분 부분을 지우고, `_on_timer`에 원래
`self._yaw_raw += omega * dt` 줄을 다시 넣으면 수동 복구 가능)

---

## 수정 5 — 2026-08-04

**배경**: 수정 2~4에서 계속 "IMU 각속도(자이로)를 직접 적분"해서 yaw를 만들어왔는데,imu를 안쓰고 있었음
(`ebimu_driver.py`가
이걸로 `msg.orientation`을 만듦). 근데 `vesc_wheel_odom.py`는 이 `orientation`을
안 쓰고 각속도만 받아서 처음부터 다시 적분하고 있었음 — 장비가 이미 계산해준
(지자기로 드리프트 보정되는) 값을 버리고 있던 셈.

**우려되는 점**: 지자기 센서는 근처 모터(VESC)/금속 섀시의 자기장에 취약해서,
모터가 돌 때 장비 자체 yaw가 오히려 튈 위험이 있음 (실측으로 확인 필요 —
수정 2에서 했던 것처럼 모터 켜고/끄고 비교 필요).

**수정**: `src/localization_layer/scripts/vesc_wheel_odom.py`,
`src/localization_layer/launch/localization_launch_common.py`

새 파라미터 `yaw_source`로 3가지 방식 중 선택 가능하게 만듦:

| 값 | 방식 | 장점 | 단점 |
|---|---|---|---|
| `'gyro'` | 각속도만 직접 적분 | 지자기 간섭 영향 없음 | 장기 드리프트 누적 |
| `'orientation'` | EBIMU 자체 지자기 융합 yaw 그대로 사용 | 장기 드리프트 없음 | 모터 자기간섭에 순간적으로 튈 위험 |
| `'fused'` (채택) | 평소엔 자이로 적분 + `yaw_fusion_tau_sec`(기본 5초)만큼 아주 느리게 orientation 쪽으로 당김 | 둘의 장점만 취함 — 순간 튐은 느린 보정이라 거의 안 묻고, 장기 드리프트는 서서히 교정됨 | 없음 (제일 안전한 선택) |

- `_on_imu()`에서 `yaw_source == 'orientation'`이면 `msg.orientation`에서
  yaw만 추출(`_yaw_from_quat`)해서 `self._yaw_raw`에 바로 대입.
- `'fused'`면 기존처럼 자이로를 매 메시지 적분하면서, 동시에
  `(dt / yaw_fusion_tau_sec) * (orientation_yaw - yaw_raw)`만큼만 살짝
  보정 방향으로 당김 (비례 보정, PI 필터의 P항과 유사).
- 실제 로컬라이제이션 launch(`localization_launch_common.py`)는
  **`yaw_source: 'fused'`, `yaw_fusion_tau_sec: 5.0`으로 설정** — 지금부터
  이 방식이 적용됨.

**참고 — 조향에 미치는 실질 영향은 제한적**: Stanley는 `/odom`이 아니라
Cartographer의 `map→base_link` TF만 읽고, 그 TF의 실시간 회전 예측은
Cartographer가 `/imu/data`를 직접(우리 `/odom`을 거치지 않고) 처리함. 우리
`/odom`의 yaw는 포즈그래프 백엔드에 낮은 가중치(`odometry_rotation_weight=300`,
라이다 쪽 `1e5`의 1/333)로만 반영됨. 그래서 `yaw_source`를 뭘로 하든 조향에는
큰 영향이 없고, 이건 `/odom`의 장기적 정확도(디버깅/보조 제약용) 개선 목적.

**확인 필요**: `colcon build --packages-select localization_layer` 후 재시작.
모터 회전 중/정지 중 각각 `ros2 topic echo /odom --field pose.pose.orientation`
값이 튀는지 확인 — `fused`라 짧은 튐은 거의 안 보여야 정상. 그래도 튀면
아래처럼 `gyro`로 되돌릴 것.

**되돌리는 법 (그래도 이상하면)**:
`localization_launch_common.py`에서 `'yaw_source': 'fused'`를
`'yaw_source': 'gyro'`로 한 줄만 바꾸면 즉시 제일 안전한 방식(수정 4, 순수
자이로 적분)으로 복귀. 코드 자체를 되돌리려면:
```bash
git checkout -- src/localization_layer/scripts/vesc_wheel_odom.py \
  src/localization_layer/launch/localization_launch_common.py
```

---

## 수정 6 — 2026-08-04

**배경**: 수정 4(오돔 100Hz 적분) + `lidar_scan_frequency:=25.0` 테스트까지
했는데도 **고속 코너에서 맵이 계속 틀어짐** 확인 (`tegrastats` CPU는 확인 중).
우선순위 3번째였던 "스캔매칭 탐색 범위 넓히기" 진행.

**수정**: `src/localization_layer/config/cartographer_2d_localization.lua`

| 파라미터 | 이전 | 이후 |
|---|---|---|
| `real_time_correlative_scan_matcher.linear_search_window` | 0.30 (±15cm) | 0.42 (±21cm) |
| `real_time_correlative_scan_matcher.angular_search_window` | 22° | 30° |

**이유**: 스캔 간격(현재 25Hz면 0.04초) 동안 3~3.5m/s로 이동하면 12~14cm
이동 + 예측 오차분까지 더해지면 기존 ±15cm 여유가 빠듯했음. 탐색 범위를
넓혀서 예측이 조금 부정확해도 실제 벽을 놓치지 않게 함.

**트레이드오프**: 탐색 범위가 넓어지면 스캔매칭 연산량이 약간 늘고(허용 범위 내
후보를 더 넓게 훑음), 아주 드물게 비슷하게 생긴 다른 구간과 헷갈릴 여지도
이론적으로는 소폭 늘어남 (수정 1에서 다룬 문제와 같은 종류). 그래도 지금은
`min_score=0.62`로 이미 조여둔 상태라 위험은 낮다고 판단.

**확인 필요**: `colcon build --packages-select localization_layer` 후 재시작,
3~3.5m/s 코너 재테스트. 그래도 안 되면 다음 후보는 `odometry_rotation_weight`
(현재 300, 수정 5로 `/odom` yaw 신뢰도가 개선됐으니 소폭 상향 검토 — 1000
정도부터 조금씩) 또는 물리적 타이어 슬립 가능성 점검.

**되돌리는 법**:
```bash
git diff HEAD -- src/localization_layer/config/cartographer_2d_localization.lua
```
위 표의 "이전" 값으로 되돌리면 됨.

---

## 수정 7 — 2026-08-04 (시간축/타임스탬프 버그)

**패키지 위치**: `src/sllidar_ros2/src/sllidar_node.cpp` (라이다 드라이버 원본,
localization_layer 아님 — 로컬라이제이션이 이 타임스탬프를 직접 쓰기 때문에 여기 같이 남김)

**증상**: 수정 4/6을 거쳐도 고속 코너에서 위치 틀어짐이 계속 남아있었음.

**원인**: `publish_scan()`에 LaserScan의 `header.stamp`로 "스캔 **시작** 시각"
(`start_scan_time`)을 넘기고 있었음. Cartographer는 이 타임스탬프를 "스캔 **마지막
포인트** 시각"으로 해석하기 때문에, 실제로는 한 바퀴 스캔 시간(약 25ms@40Hz)만큼
과거로 찍히는 셈이었음. IMU는 지연이 거의 없이(~1ms) 들어오는데 라이다만 이렇게
늦게 찍히니, 둘을 시간 맞춰 비교(스캔매칭/pose 예측)할 때마다 어긋남 발생.
직선에서는 위치 변화가 적어 안 보이지만, **코너에서는 방향이 빠르게 바뀌니 이
시간 오차가 곧바로 헤딩 오차로 증폭**됨 — 지금까지 봐온 "고속 코너에서만 유독
틀어지는" 증상과 정확히 일치.

**수정**: `publish_scan()` 호출 3곳 전부 `start_scan_time` → `end_scan_time`으로 변경.

**관련 후속 조치**: 라이다 드라이버 레벨의 근본 타이밍 버그였던 거라, CPU 부담
때문에 미뤄뒀던 40Hz 복귀도 다시 시도 중 (`lidar_scan_frequency`: 20 → 40,
`mapping_sensor_bringup_launch.py` + `localization_launch_common.py`). 근본
원인이 이 타임스탬프였다면 40Hz에서도 안정적일 가능성이 있음 — Jetson 부하 확인 필요.

**⚠️ 빌드 필요**: 이건 C++ 코드라 lua/launch 파일과 다르게 **꼭 빌드해야 반영돼요**:
```bash
colcon build --packages-select sllidar_ros2
```

**확인 필요**: 빌드 후 재시작해서 고속 코너 재테스트, `ros2 topic hz /scan`으로
40Hz 유지되는지, CPU 여유(`tegrastats`) 확인.

**되돌리는 법**:
```bash
git diff HEAD -- src/sllidar_ros2/src/sllidar_node.cpp \
  src/localization_layer/launch/mapping_sensor_bringup_launch.py \
  src/localization_layer/launch/localization_launch_common.py
```

---

## 수정 8 — 2026-08-04 (검색범위 되돌림 + 코어 재조정) → **되돌림 (아래 참고)**

**⚠️ 적용 후 바로 원상복구함**: 적용 직후 "초기 위치를 아예 못 잡는다"는 증상이
나와서 사용자가 원복 요청. 근데 실제로 로그 확인해보니 원인은 이 수정이
아니라 **Foxglove의 Display Frame이 `map`이 아니라 `base_link`로 설정돼 있어서
`/initialpose`가 계속 거부되고 있던 것**(`Ignoring /initialpose frame_id='base_link'`)
— 예전에 한 번 겪었던 것과 똑같은 문제. 다만 "혹시 몰라서" 안전하게 원복함.
아래 내용은 시도했던 기록으로 남겨둠 (지금은 미적용 상태).

**배경**: 수정 6~7 이후에도 실주행 중 위치가 계속 틀어짐. 새 맵(190909)으로
바뀐 뒤 처음 겪는 문제라 원인 후보가 많았음 — CPU 부하인지, 매칭 파라미터인지
구분 필요했음.

**CPU/램 실측 (전체 스택 켠 상태)**:
```
코어: 6개, load average: 4.74/6 (여유 있음)
메모리: 3.7GB available, 스왑 0B 사용
```
→ **CPU/메모리는 문제가 아님**을 확인. 이전에 의심했던 "코어 3개+40Hz라 부하일
것"이라는 가설은 실측으로 기각됨.

**그래서 재검토**: CPU가 괜찮다면 원인은 "계산량 부족"이 아니라 "매칭이 잘못된
후보를 확신 있게 고르는 것"(품질 문제)일 가능성이 큼. 의심 지점:

```
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window
```
이 값이 0.42(수정6, 3~3.5m/s 실측 근거 있음) → 0.5(10m/s급 대비 "방어적
마진", 실측 근거 없이 미리 늘려둔 값)로 되어 있었음. 트랙이 반복적인 루프
형태라, 탐색 범위가 넓을수록 다른 구간과 헷갈릴 위험이 있음(수정 1과 같은
종류의 문제, 이번엔 로컬 스캔매칭 레벨에서).

**수정**: `src/localization_layer/config/cartographer_2d_localization.lua`

| 파라미터 | 이전 | 이후 | 이유 |
|---|---|---|---|
| `real_time_correlative_scan_matcher.linear_search_window` | 0.5 | 0.42 | 실측 근거 없던 마진 제거, 실측 검증됐던 값으로 복귀 |
| `MAP_BUILDER.num_background_threads` | 3 | 4 | CPU 여유 확인됐으니 살짝 상향 |

**확인 필요**: 재시작 후 고속 코너 재테스트. 위치 틀어짐이 줄어드는지 확인.
그래도 안 되면 다음 후보는 새 맵(190909) 자체의 품질(Foxglove로 벽선이
깔끔한지 육안 확인) 또는 `stanley_waypoint_follow_node.py`의
`reverse_track_direction`(조향 방향, 위치추정과는 별개 문제).

**되돌리는 법**:
```bash
git diff HEAD -- src/localization_layer/config/cartographer_2d_localization.lua
```

---

## 수정 9 — 2026-08-14

**배경**: (다른 팀원이 완전히 새로 짠 "Jetson lite" 버전 위에서 진행) 팀원이
"yaw를 너무 크게 받는 것 같다"고 보고함. 로컬라이제이션 매칭 스펙(정확도) 상향
겸 서브맵 유지 개수 상향 요청.

**수정**: `src/localization_layer/config/cartographer_2d_localization.lua`

| 파라미터 | 이전 | 이후 | 이유 |
|---|---|---|---|
| `TRAJECTORY_BUILDER.pure_localization_trimmer.max_submaps_to_keep` | 2 | 4 | 참고할 submap을 더 많이 유지해서 매칭 품질 상향 |
| `TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight` | 30.0 | 15.0 | 30이면 prior(odom/IMU) 회전에서 안 벗어나려는 힘이 강해서, prior의 yaw가 과대해도 라이다가 잘 못 눌러줌 → 낮춰서 라이다가 회전을 더 적극적으로 보정하게 함 |

**참고**: `max_submaps_to_keep`을 올리면 메모리/매칭 계산량이 약간 늘어남.
`num_background_threads=2`로 낮게 잡혀있는 "Jetson lite" 구성이라, CPU 여유
없으면 이 상향이 부담될 수 있음 — 재시작 후 `top`으로 CPU 확인 권장.

**확인 필요**: 재시작 후 회전(코너/제자리 회전)할 때 yaw가 이전보다 안정적으로
나오는지, `ros2 topic echo /imu/data --field angular_velocity` 및 실제 화면상
헤딩 변화로 비교. 서브맵 4개로 늘린 뒤 CPU 부담 없는지도 같이 확인.

**되돌리는 법**:
```bash
git diff HEAD -- src/localization_layer/config/cartographer_2d_localization.lua
```

---

## 수정 10 — 2026-08-15

**배경**: (다른 팀원이 다시 새로 튜닝한 버전 위에서 진행) "2m/s까지는 잘 되는데
3m/s부터 깨진다"는 증상 + 회전할 때 계속 틀어지는 문제. 이 버전은 전체적으로
prior(odom+IMU 예측)를 강하게 믿고 라이다 보정은 좁게 막아두는 방향으로
튜닝되어 있었음 (`angular_search_window=12°`, `rotation_delta_cost_weight=20`,
`odometry_rotation_weight=1e4` 등).

**왜 3m/s부터 깨지는지**: 40Hz라 스캔 간격은 0.025초로 짧지만, CPU가
load average 6대(6코어 거의 꽉 참)라 처리가 살짝 밀릴 수 있음. 밀린 시간 동안
고속 코너에서 실제로 도는 각도가 좁아진 탐색범위(12°)를 넘어버리면 못 찾음.
저속에서는 같은 시간에 도는 각도가 작아서 12° 안에 들어와 괜찮았던 것.
게다가 odom의 회전값은 순수 자이로 적분(지자기 보정 없음)이라 시간이 지날수록
오차가 쌓이는데, 이 오차를 라이다가 고쳐줄 여지(탐색범위·가중치)까지 같이
좁혀놔서 누적 오차가 그대로 반영되기 쉬운 상태였음.

**수정**: `src/localization_layer/config/cartographer_2d_localization.lua`

| 파라미터 | 이전 | 이후 | 이유 |
|---|---|---|---|
| `real_time_correlative_scan_matcher.angular_search_window` | 12° | 25° | 고속 코너에서 처리 지연 있어도 실제 회전량을 탐색범위 안에 들어오게 |
| `real_time_correlative_scan_matcher.rotation_delta_cost_weight` | 20.0 | 5.0 | prior(odom 회전)에서 벗어나기 쉽게 해서 라이다가 회전을 더 자유롭게 보정 |
| `POSE_GRAPH.optimization_problem.odometry_rotation_weight` | 1e4 | 5e2 | odom 회전(누적 오차 있음)을 다시 보조 역할로만 사용 |

**나머지 값은 그대로 둠**: `max_submaps_to_keep=2`, `linear_search_window=0.35`,
`ceres_scan_matcher.rotation_weight=55.0` 등 나머지 튜닝은 팀원이 최근에
다시 잡아둔 값이라 손대지 않음. 이번엔 회전 관련 3곳만 조정.

**확인 필요**: 재시작 후 3m/s 이상으로 코너 재테스트. 여전히 깨지면 CPU
부담(load average 6대) 자체를 먼저 줄여야 할 수도 있음 (다른 노드 정리 또는
`num_background_threads` 조정 — 단 이건 이미 2로 낮게 잡혀 있어서 여유 적음).

**되돌리는 법**:
```bash
git diff HEAD -- src/localization_layer/config/cartographer_2d_localization.lua
```
위 표의 "이전" 값으로 되돌리면 됨.

---

## ⚠️ 2026-08-15 — 지금까지 실측으로 확인된 "제일 나은" 조합 (함부로 갈아엎지 마세요)

**밤새 여러 사람이 이 파일을 계속 다른 방향으로 고쳐왔습니다.** "odom을 약하게, 라이다를
강하게", "탐색범위 넓게" 등 이론적으로 맞아 보이는 방향들을 실제로 다 테스트해봤는데,
**아래 값들이 그 어떤 조합보다 실제 주행에서 제일 안정적이었습니다.**

```lua
TRAJECTORY_BUILDER.pure_localization_trimmer.max_submaps_to_keep = 2
options.tracking_frame = "imu_link"
options.published_frame = "base_link"
options.provide_odom_frame = true
options.use_odometry = true
options.pose_publish_period_sec = 0.05
options.submap_publish_period_sec = 2.0
MAP_BUILDER.num_background_threads = 2
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 80.0
TRAJECTORY_BUILDER_2D.max_range = 20.0
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.10
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.35
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(12.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 20.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 30.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 8.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 55.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 6
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.06
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60
POSE_GRAPH.optimize_every_n_nodes = 80
POSE_GRAPH.global_sampling_ratio = 0.003
POSE_GRAPH.constraint_builder.sampling_ratio = 0.04
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.82
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 1.2
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(12.)
POSE_GRAPH.global_constraint_search_after_n_seconds = 15.
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e4
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e4
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 4
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 2
POSE_GRAPH.max_num_final_iterations = 4
```

**시도했지만 이것보다 별로였던 조합들** (또 시도하지 마세요):
- odom 회전 가중치를 낮추고(`odometry_rotation_weight` 낮춤) 라이다를 더 믿게 한 조합
- `angular_search_window`를 20~25도로 넓힌 조합

**만약 이 파일을 또 고치고 싶다면**:
1. 먼저 이 섹션을 읽고, 왜 이 값들인지 이해하고 시작할 것
2. 한 번에 하나씩만 바꾸고, 최소 여러 번 반복 주행 테스트 후 결론 낼 것
3. 바꾼 이유와 결과를 반드시 이 파일에 새 "수정 N" 항목으로 남길 것
4. 이 조합으로 안 되면, 최소한 **되돌아올 수 있는 값이 바로 이 섹션**이라는 걸 기억할 것

---

## 수정 15 — 2026-08-22 (S자 / 90° 코너 가끔 밀림 미세 조정)

**증상**: 전체적으로 매우 안정적인데 S자·90° **고속 회전(yaw rate 큼)** 구간에서 가끔 조금 틀어짐.

**원인 추정** (저속 apex 아님 — 차량 yaw rate가 빠른 경우):
- `angular_search_window`가 고속 회전+CPU 지연 대비 좁음 (40Hz, ω~4rad/s → 스캔당 ~6°)
- `rotation_delta_cost_weight` 높으면 prior 회전에 붙어 라이다 yaw 보정 지연
- `max_range` 넓으면 S자에서 평행 벽 오매칭

**수정**: `cartographer_2d_localization.lua` + `localization_launch_common.py`

| 파라미터 | 이전 | 이후 |
|---|---|---|
| `max_range` | 25.0 | **18.0** |
| `angular_search_window` | 12° | **18°** |
| `rotation_delta_cost_weight` | (28 상속) | **10** |
| `translation_delta_cost_weight` | 18 | **15** |
| `ceres rotation_weight` | 55 | **65** |
| `motion_filter.max_angle_radians` | (없음) | **1.5°** |
| `optimize_every_n_nodes` | 6 | **3** |
| `motion_filter` time / dist | 0.03s / 0.04m | **0.025s / 0.03m** |
| `translation_delta_cost_weight` | 18 | **12** |
| `ceres occupied / translation` | 24 / 14 | **28 / 11** |
| `vesc_wheel_odom publish_on_imu` | false (50Hz) | **true (~100Hz)** |
| pose graph `fast_correlative angular` | 10° | **14°** |
| `min_speed_for_yaw_mps` | 0.08 | **0.03** |

**의도적으로 안 건드린 것**: `linear_search_window=0.22`, `odometry_rotation_weight=1e4` (README 실측)

**되돌리는 법**: 위 표 "이전" 값으로 `cartographer_2d_localization.lua` 복구.

---

## 수정 16 — 2026-08-22 (CPU 과부하 + 코너 드리프트 재균형)

**증상**: 수정15(즉시 보정) 이후 CPU 부족 + 코너에서 여전히 가끔 밀림.

**분석**: `optimize_every_n_nodes=3`, `motion_filter` 과민, `angular=18°`, `background_threads=4`
→ pose graph·correlative 연산 폭증. 코너 보정은 **local SLAM**이 담당해야 PG가 아님.

**수정**: `cartographer_2d_localization.lua` — CPU↓, 코너는 local 매칭 유지

| 파라미터 | 수정15 | 수정16 |
|---|---|---|
| `num_background_threads` | 4 | **2** |
| `optimize_every_n_nodes` | 3 | **8** |
| `motion_filter` | 0.025s/0.03m | **0.03s/0.04m** |
| `angular_search_window` | 18° | **16°** |
| `voxel_filter_size` | 0.08 | **0.10** |
| `sampling_ratio` | 0.35 | **0.22** |
| `ceres max_iterations` | (기본) | **8** |
| `max_range` | 18 | **18** (유지) |
| `rotation_delta_cost_weight` | 10 | **11** |

**CPU 더 필요하면**: `lidar_scan_frequency:=35.0` (launch 인자, 장애물 노드와 별도 터미널이면 OK)

---

## 수정 11 — 2026-08-15 (라이다 TF 회전값 + 맵핑 Hz)

**배경**: Foxglove에서 맵핑 중 constraint 시각화(무지개색 삼각형들)가 라이다 벽
스캔이랑 안 맞고 어긋나 보이는 문제. "라이다 앵글 오프셋을 예전엔 π로 뒀다가
0으로 바꿨던 것 같다"는 제보 + 실측 화면에서 좌우 반전으로 보임.

**수정 1) `src/tf_manager_cpp/src/sensor_static_tf.cpp`** — `publish_lidar_tf()`가
`base_link → laser` 회전을 **항상 0(회전 없음)으로 하드코딩**하고 있었음
(바로 아래 `publish_imu_tf()`는 `imu_roll/pitch/yaw` 파라미터로 조정 가능한데
라이다만 이 보정이 빠져 있었음). IMU와 동일한 패턴으로
**`lidar_roll`/`lidar_pitch`/`lidar_yaw` 파라미터 추가, `lidar_yaw` 기본값 π(180°)**
로 설정.

```cpp
// 변경 전: rotation.x/y/z=0, w=1 고정
// 변경 후:
const double roll = declare_parameter("lidar_roll", 0.0);
const double pitch = declare_parameter("lidar_pitch", 0.0);
const double yaw = declare_parameter("lidar_yaw", M_PI);
tf2::Quaternion q; q.setRPY(roll, pitch, yaw);
```

**⚠️ 미해결 — 좌우 반전(mirror)과 회전(rotation)은 다른 문제임**: 실측 화면이
"좌우 반전"으로 보인다면, 방금 넣은 `lidar_yaw`(회전)로는 근본적으로 못 고침
(회전은 손잡이 방향을 안 바꾸지만 거울상은 바꿈). 진짜 좌우 반전이면
`src/localization_layer/launch/mapping_sensor_bringup_launch.py:171`의
**`inverted` 파라미터(현재 기본값 `false`)를 `true`로 바꿔야 할 가능성이 높음.**
또한 같은 파일 173~176번째 줄에 별도로 **`angle_offset`**(기본 0, 설명:
"pi면 좌우 거울이 됨, 드라이버가 이미 pi-theta 보정함")이라는 파라미터가
있는데 이건 `lidar_yaw`(TF)와 다른 레이어(드라이버 자체 각도 보정)라서, 실측
테스트할 땐 이 값도 같이 확인 필요. **다음 실측 시 확인할 것: `inverted`,
`angle_offset`, `lidar_yaw` 세 가지를 하나씩만 바꿔가며 어느 게 실제 원인인지
구분.**

**수정 2) 맵핑 LiDAR Hz — 40 유지 (20 테스트 후 되돌림)**: 맵핑 중
`range_data_collator.cc:82 Dropped N earlier points` 경고가 대량(수백 개
단위)으로 발생 + `Remaining work items in queue: 1076`까지 확인 → CPU 과부하
의심되어 40→20Hz로 테스트 시도. 그러나 `cartographer_mapping_launch.py`의
기존 주석에 **"20Hz는 회전 중 한 스캔이 더 오래 걸려서 코너가 휘어 보이는
문제 때문에 40으로 올렸었다"**는 기록이 있어, **최종적으로 40Hz로 재복귀
결정** (`mapping_sensor_bringup_launch.py:179`, `cartographer_mapping_launch.py:379`
둘 다 `default_value='40.0'`).

**즉 CPU 과부하(Dropped points)와 코너 스캔 왜곡, 두 문제가 Hz를 반대 방향으로
당기고 있는 상태.** 아직 근본 해결 안 됨 — 다음 후보:
- 맵핑용 lua(`cartographer_2d_mapping_imu_lidar_no_odom.lua`)의 탐색범위/스레드
  수는 원래도 좁고 낮아서(0.16/8°, 3스레드) 40Hz에서 왜 이렇게 밀리는지 재확인 필요
  (로컬라이제이션 lua와 달리 이 파일 자체는 안 건드림)
- 또는 Dropped points가 실제 맵 품질에 얼마나 영향 주는지 실측으로 확인 후,
  "버텨지는 정도면 그냥 40 유지"로 갈 수도 있음

**확인 필요**: 다음 맵핑 세션에서 (1) 라이다 방향(좌우반전 여부, `inverted`/
`angle_offset`/`lidar_yaw` 조합), (2) 40Hz에서의 Dropped points 빈도와 실제
맵 품질 영향, 둘 다 재확인.

**빌드**:
```bash
cd ~/f1tenth_ajou
colcon build --packages-select tf_manager_cpp localization_layer
source install/setup.bash
```

**되돌리는 법**:
```bash
git diff HEAD -- src/tf_manager_cpp/src/sensor_static_tf.cpp \
  src/localization_layer/launch/mapping_sensor_bringup_launch.py \
  src/localization_layer/launch/cartographer_mapping_launch.py
```

## 수정 12 — 2026-08-15 (좌우반전 원인 재조사 → IMU 축 하드웨어 결함으로 결론)

**배경**: 수정11에서 남겨둔 "좌우반전 미해결" 이슈를 실측으로 계속 추적.

**1) `inverted` 인자 이름 버그 발견 + 수정**: `cartographer_mapping_launch.py`
최상위에서 `inverted:=true`로 넘겨도 실제론 무시되고 있었음 — 이 launch 파일이
선언하는 인자 이름은 `inverted`가 아니라 **`lidar_inverted`**
(`cartographer_mapping_launch.py:363`, 내부적으로
`mapping_sensor_bringup_launch.py`의 `inverted`로 전달). `lidar_inverted:=true`로
정정 후 `ros2 param get /sllidar_node inverted` → `True` 확인. **근데 이렇게
정확히 켜도 좌우반전은 그대로** — 라이다 드라이버 쪽은 원인이 아닌 걸로 결론.

**2) 중복 프로세스 / Hz 불일치 가설 기각**: `ps aux | grep sllidar_node` 확인 결과
프로세스 1개만 존재 (respawn 중복 아님). `ros2 topic hz /scan` 실측 결과
평균 39.8Hz로 설정값(40Hz)과 일치 — 로그에 찍히던 `scan rate: ~398Hz`는
Cartographer 내부 스캔 서브디비전 처리 빈도였을 뿐 실제 토픽 속도 아님
(정상 동작, 에러 아님).

**3) 진짜 원인 — IMU 하드웨어 결함**: 사용자 확인 결과, 최근 물리적으로 위치가
바뀐 건 라이다가 아니라 **IMU**였고, "하드웨어 오류로 X축이 원래 의도(오른쪽)와
반대로 왼쪽을 향하고 있다"는 사실이 새로 확인됨. `src/ebimu_pkg/ebimu_pkg/ebimu_driver.py:14-16`의
축 변환 가정("칩 +X 오른쪽, +Y 뒤, +Z 아래")이 실제 하드웨어와 어긋난 상태.
Y/Z는 정상, X만 반전된 상태라 이건 회전이 아니라 **진짜 거울반사(determinant -1)**
이고, 이게 자이로 yaw 부호까지 반전시켜서 SLAM 궤적 추정이 반대로 돌고
그 결과 맵이 거울처럼 보였던 것으로 결론.

**⚠️ 미적용 — 다음에 할 것**: `src/ebimu_pkg/ebimu_pkg/ebimu_driver.py:25-26`의
```python
def _chip_vec_to_ros(x, y, z):
    return (-y, -x, -z)   # 현재
```
를
```python
def _chip_vec_to_ros(x, y, z):
    return (-y, x, -z)    # X만 부호 반전 (하드웨어 결함 반영)
```
로 고쳐야 함 (accel/gyro 공용 함수). **아직 코드 수정 안 함** — 다음 세션에서
적용 후 반드시 라이브 테스트로 검증할 것: `ros2 run ebimu_pkg ebimu_driver ...`
켜고 `ros2 topic echo /imu/data --field angular_velocity.z` 보면서 차 앞부분을
왼쪽으로 돌렸을 때 양수 나오는지 확인. 또한 `_ROS_FROM_CHIP_Q`(orientation
쿼터니언, 17~22번째 줄)도 같은 결함을 반영해서 같이 고쳐야 하는지 재검토 필요
(현재는 벡터 변환 함수만 분석함, 쿼터니언 쪽은 미검토).

**4) 맵핑 Hz 40 → 20 재복귀**: 40Hz에서 `range_data_collator.cc:82 Dropped N
earlier points`와 `sensor_bridge.cpp:211 Ignored subdivision...` 경고가 계속
반복 발생 확인 (`/scan` 메시지 도착 간격이 20~50ms로 들쭉날쭉해서 Cartographer의
스캔 서브디비전 타임스탬프 보간이 깨지는 것으로 추정, 근본 원인 미해결 —
IMU 버스트 타이밍 문제와 같은 계열로 의심됨). 수정11에서 "코너 왜곡 때문에
40 유지" 결정했었지만, 사용자 지시로 **20Hz로 재복귀**
(`cartographer_mapping_launch.py:379`, `mapping_sensor_bringup_launch.py:179`
둘 다 `default_value='20.0'`). 코너 스캔 왜곡 재발 여부 다음 맵핑 세션에서 확인 필요.

**빌드**: 둘 다 심볼릭 링크 설치라 빌드 불필요 (launch 파일 Python, 저장 즉시 반영).
`ebimu_driver.py` 수정 적용 시에도 마찬가지로 빌드 불필요 (ament_python 심볼릭 링크).

**되돌리는 법**:
```bash
git diff HEAD -- src/localization_layer/launch/cartographer_mapping_launch.py \
  src/localization_layer/launch/mapping_sensor_bringup_launch.py \
  src/ebimu_pkg/ebimu_pkg/ebimu_driver.py
```

## 수정 13 — 2026-08-15 (`lidar_yaw` 기본값 π → 0 되돌림)

**배경**: 수정11에서 넣은 `lidar_yaw=π`가 실제 라이다 실측이 아니라, 수정12에서
찾아낸 IMU 축 결함(아직 미수정)과 뒤섞인 진단에서 나온 값일 수 있다는 게
밝혀짐 — 검증 안 된 값이라 판단, 사용자 지시로 0(회전 없음)으로 되돌림.

**수정**: `src/tf_manager_cpp/src/sensor_static_tf.cpp` — `publish_lidar_tf()`의
`declare_parameter("lidar_yaw", M_PI)` → `declare_parameter("lidar_yaw", 0.0)`.
컴파일 패키지라 `colcon build --packages-select tf_manager_cpp` 완료함.

**⚠️ 중요 — 맵 일관성 깨짐**: 22:38(π 빌드) ~ 이 수정 사이에 만들어진 맵
(`20260815_224047`~`224547` 등)은 라이다가 π로 회전된 좌표계 기준으로
저장됨. 이 맵들에 지금(yaw=0) 상태로 로컬라이제이션 걸면 스캔이 180도
어긋나서 매칭 실패 가능성 높음 — **재사용하려면 반드시 새로 맵핑부터
다시 떠야 함.**

**확인 필요**: 다음 맵핑 세션에서 yaw=0이 실제로 맞는지 Foxglove로 재검증.
IMU 축 버그(수정12, `ebimu_driver.py` 미적용)를 먼저 고치고 나서 판단하는 게
정확함 — 순서 바뀌면 또 원인 구분 안 됨.

**되돌리는 법**:
```bash
git diff HEAD -- src/tf_manager_cpp/src/sensor_static_tf.cpp
# 원상복구(π로) 하려면 M_PI로 다시 바꾸고:
cd ~/f1tenth_ajou && colcon build --packages-select tf_manager_cpp && source install/setup.bash
```

## 수정 14 — 2026-08-16 (`angle_offset` 원본값 π로 복원)

**배경**: 원본 커밋(`7768a99`) 대비 uncommitted 상태로 `angle_offset` 기본값이
`π`에서 `0.0`으로 바뀌어 있던 게 git diff로 확인됨 — 누가/언제 바꿨는지는
특정 안 됨(이번 세션에서 Claude가 바꾼 게 아님, 대화 시작 전부터 이미 이 상태).
사용자 지시로 원본값(π)으로 복원.

**수정**: 두 파일 모두 `angle_offset` 기본값 `0.0` → `3.141592653589793`(π)로
복원, description도 "pi면 좌우 거울"이라는 경고 문구(0이 기본이던 시절 문구,
지금 값이랑 모순됨) 대신 사실 기술로 정리:
- `src/localization_layer/launch/mapping_sensor_bringup_launch.py:173-178`
- `src/sensor_layer/launch/sensor_layer_launch.py:59`

둘 다 Python launch 파일(심볼릭 링크 설치)이라 빌드 불필요, 저장 즉시 반영.

**참고**: `sllidar_ros2/launch/sllidar_t1_launch.py:23`에도 `angle_offset`
기본값 `0.0`이 있으나 이건 git 원본과 동일(아무도 안 건드림) + 실제 launch
체인에서 안 쓰이는 벤더 예제 파일로 확인됨 — 그대로 둠.

**⚠️ 미해결**: 이 값(드라이버 레벨 각도 보정)과 `sensor_static_tf.cpp`의
`lidar_yaw`(TF 레벨 회전, 수정13에서 0으로 되돌림)는 서로 다른 레이어라 각각
독립적으로 맞는지 확인 필요 — 아직 "이 조합으로 맵/스캔이 정확히 정렬되는지"
실측 검증 안 됨.

**되돌리는 법**:
```bash
git diff HEAD -- src/localization_layer/launch/mapping_sensor_bringup_launch.py \
  src/sensor_layer/launch/sensor_layer_launch.py
```
