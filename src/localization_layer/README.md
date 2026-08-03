# 명령어

cd ~/f1tenth_ajou
source install/setup.bash

ros2 launch localization_layer cartographer_mapping_launch.py

ros2 launch localization_layer cartographer_localization_launch.py

ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765



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
