"""M-of-N 확정 게이트 검증.

핵심 요구사항 두 개가 동시에 성립해야 한다.
  1. 매 프레임 잡히는 실제 장애물은 빠르게(고속 주행에 지장 없이) 확정된다.
  2. 띄엄띄엄 잡히는 라이다 노이즈는 아무리 오래 지나도 확정되지 않는다.
"""

from path_following.detection_confirm import HitHistory

SCAN_HZ = 40.0
WINDOW = 6
MIN_HITS = 4


def _run(pattern, *, window=WINDOW, min_hits=MIN_HITS):
    """pattern[0] 으로 트랙을 만들고 나머지를 순서대로 먹인다.

    확정된 프레임 인덱스(0-based)를 돌려준다. 끝까지 확정 안 되면 None.
    """
    h = HitHistory(window, min_hits, initial_hit=bool(pattern[0]))
    if h.confirmed:
        return 0
    for i, hit in enumerate(pattern[1:], start=1):
        if h.update(bool(hit)):
            return i
    return None


def test_real_obstacle_confirms_within_min_hits():
    """매 프레임 검출되면 정확히 min_hits 번째 프레임에 확정된다."""
    idx = _run([1] * 20)
    assert idx == MIN_HITS - 1


def test_confirm_latency_is_affordable_at_race_speed():
    """확정 지연이 7m/s 에서 탐지 사거리의 10% 를 넘지 않아야 한다."""
    idx = _run([1] * 20)
    latency_s = (idx + 1) / SCAN_HZ
    travel_m = latency_s * 7.0
    assert latency_s <= 0.15
    assert travel_m / 11.0 <= 0.10


def test_single_frame_noise_never_confirms():
    """한 프레임만 뜨는 노이즈는 확정되지 않는다."""
    assert _run([1] + [0] * 30) is None


def test_two_frame_noise_never_confirms():
    assert _run([1, 1] + [0] * 30) is None


def test_flickering_noise_never_confirms_even_over_long_time():
    """예전 버그의 핵심 재현.

    누적 관측시간 방식은 4프레임에 한 번 깜빡여도 시간이 쌓여 결국
    확정됐다. 창 방식은 200프레임(5초)이 지나도 확정되지 않아야 한다.
    """
    pattern = ([1] + [0] * 3) * 50
    assert _run(pattern) is None


def test_every_other_frame_noise_never_confirms():
    """2프레임에 한 번 = 창 6칸 중 3회. min_hits 4 미만이므로 기각."""
    assert _run([1, 0] * 50) is None


def test_mostly_detected_obstacle_confirms():
    """6프레임 중 4회 잡히는 흐릿한 실제 장애물은 확정된다."""
    idx = _run([1, 1, 0, 1, 0, 1] * 5)
    assert idx is not None


def test_confirmation_latches_through_occlusion():
    """한 번 확정되면 이후 계속 놓쳐도 확정이 풀리지 않는다.

    회피 도중 장애물이 잠깐 가려졌다고 발행이 끊기면 경로가 되감긴다.
    """
    h = HitHistory(WINDOW, MIN_HITS)
    for _ in range(MIN_HITS - 1):
        h.update(True)
    assert h.confirmed
    for _ in range(50):
        h.update(False)
    assert h.confirmed


def test_min_hits_one_restores_legacy_behaviour():
    """롤백 스위치. confirm_min_hits=1 이면 첫 관측에 즉시 확정된다."""
    h = HitHistory(WINDOW, 1)
    assert h.confirmed
    assert _run([1] + [0] * 10, min_hits=1) == 0


def test_min_hits_clamped_to_window():
    """min_hits 가 창보다 크면 창 크기로 묶여 영원히 미확정이 되지 않는다."""
    h = HitHistory(4, 99)
    for _ in range(3):
        h.update(True)
    assert h.confirmed


def test_hits_counts_only_within_window():
    h = HitHistory(4, 4, initial_hit=True)
    for _ in range(3):
        h.update(True)
    assert h.hits == 4
    for _ in range(4):
        h.update(False)
    assert h.hits == 0
