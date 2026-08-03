# ebimu_pkg — EBIMU 시리얼 드라이버

EBIMU를 시리얼로 읽어 `/imu/data` (`sensor_msgs/Imu`)로 발행하는 노드.

---

## 2026-08-03 수정: IMU 메시지 burst(몰림) 현상 수정

### 증상

`ros2 topic hz /imu/data` 확인 결과, 평균 100Hz인데 메시지 간격이
**0.001s ~ 0.026s로 26배 차이 나게 들쭉날쭉**했음. 몇 개가 거의 동시에
몰려 왔다가, 잠깐 끊기고, 다시 몰려오는 패턴.

실차 주행에서는 **커브 구간에서 로컬라이제이션(Cartographer) 위치가
튀는 현상**으로 나타남 (Foxglove에서 base_link 주변에 부채꼴/스타버스트
모양으로 라이다 스캔이 흩어져 보임) → path_following이 잘못된 위치를
기준으로 조향 계산 → 코너에서 충돌.

### 원인

`ebimu_driver.py`의 `read_serial()`이 100Hz(10ms) 타이머로 돌면서,
그 순간 시리얼 버퍼에 쌓여 있는 프레임을 **있는 대로 전부** `process_frame()`
→ `publish()` 했음. 타이밍이 밀리면 한 틱에 여러 프레임이 쌓이고,
그게 전부 `self.get_clock().now()` 기준 timestamp로 거의 동시에 발행됨
→ 다음 틱은 새 데이터가 없어 건너뜀 → burst 패턴 발생.

Orientation-only 프레임 처리 시 각속도를
`(현재각 - 이전각) / (now - last_time)` 로 직접 미분하는데, 이 `dt`가
burst 때문에 순간적으로 아주 작게 찍히면 각속도가 스파이크로 튐.
직선에서는 각도 변화가 거의 없어 티가 안 나지만, **커브에서는 실제
각도 변화가 커서 이 오차가 그대로 증폭**됨.

### 수정 내용 (v2 — 프레임을 버리지 않는 방식으로 재수정)

처음엔 "한 틱에 여러 프레임이 쌓이면 최신 프레임 하나만 publish"로
고쳤었는데, 이러면 **실제로 센서가 측정한 값을 버리게 되는** 문제가
있었음 (타이밍을 깔끔하게 보이려고 데이터 완전성을 희생한 것 —
빠르고 간단하지만 더 정확한 방법은 아니었음).

그래서 **프레임을 하나도 버리지 않고 전부 publish하되, 한 틱에 몰려온
프레임들에 실제 샘플링 간격(`imu_sample_period_s`, 기본 0.01s = 100Hz
가정)만큼 역산한 timestamp를 균등하게 매기는 방식**으로 다시 수정함.
가장 오래된 프레임일수록 과거 시각, 가장 최신 프레임이 지금 시각을
갖도록 함.

```python
# 변경 전 (burst 그대로 — 전부 같은 timestamp로 처리됨)
for frame in complete_frames:
    self.process_frame(frame)

# 변경 후 (전부 publish, timestamp만 역산해서 균등 분배)
def _process_frame_batch(self, frames):
    now = self.get_clock().now()
    n = len(frames)
    for i, frame in enumerate(frames):
        offset = Duration(seconds=self.nominal_period_s * (n - 1 - i))
        self.process_frame(frame, stamp=now - offset)
```

`process_frame()` / `publish_imu()`에 `stamp` 파라미터를 추가해서,
이 timestamp가 orientation-only 프레임의 각속도 계산(`dt`)에도
그대로 반영되도록 함 (`ebimu_driver.py:224` 부근).

새 파라미터 `imu_sample_period_s` (기본 `0.01`) — EBIMU의 실측 평균
주기. 실제 출력 Hz가 다르면 이 값을 조정 (예: 실측이 정확히 100Hz가
아니라면 `ros2 launch ... imu_sample_period_s:=0.0095` 식으로 조정).

### 적용 방법

코드만 바꾼다고 바로 반영되지 않음. 빌드 + 노드 재시작 필요:

```bash
cd /home/nvidia/f1tenth_ajou
source /opt/ros/humble/setup.bash
colcon build --packages-select ebimu_pkg
source install/setup.bash
```

이미 떠 있는 `ebimu_driver` 노드는 재시작해야 반영됨 (로컬라이제이션
launch를 껐다 다시 켜거나, ebimu 노드만 따로 껐다 켜기).

### 확인 방법

```bash
ros2 topic hz /imu/data
```
`min`/`max`/`std dev` 간격이 이전보다 훨씬 균일해졌는지 확인
(이상적으로는 std dev가 0.003s 이하로 확 줄어야 함). 이번엔 프레임을
안 버리므로 **평균 Hz 자체는 수정 전과 거의 같아야 정상** (줄어들면
뭔가 잘못된 것).

그 다음 실차로 커브 구간 테스트 — Foxglove에서 커브 중 base_link 위치가
더 이상 튀지 않는지, 라이다 스캔이 벽에 잘 붙어 있는지 확인.

### 되돌리는 방법 (revert)

이 수정은 git으로 추적되는 파일이라 아래 명령으로 바로 원상복구 가능:

```bash
cd /home/nvidia/f1tenth_ajou
git checkout -- src/ebimu_pkg/ebimu_pkg/ebimu_driver.py
colcon build --packages-select ebimu_pkg
source install/setup.bash
```

또는 수동으로 되돌리려면, `_process_frame_batch()`를 지우고
`read_serial()` / `process_line_frames()`의 호출부를 다시 원래의
`for frame in complete_frames: self.process_frame(frame)` 루프로
바꾸면 됨 (`process_frame`/`publish_imu`의 `stamp` 파라미터는
`stamp=None` 그대로 둬도 무해함 — 안 넘기면 자동으로 `now()` 사용).
