"""단발 LiDAR 노이즈가 장애물로 승격되는 걸 막는 M-of-N 확정 게이트.

기존 두 장애물 노드는 '누적 관측 시간'(age_s / persist[3])만 보고 확정했다.
그런데 이 누적값은 미검출 프레임에서 줄지 않고, 트랙은 keep_time_s 동안
살아남는다. 그래서 몇 프레임에 한 번씩 깜빡이는 노이즈도 트랙이 죽지 않은
채 관측 시간만 계속 쌓아 결국 확정됐다. 정면에 노이즈가 끼면 FGM 이 켜지고
조향이 꺾이던 원인이 이것이다.

여기서는 '최근 N 프레임 중 M 프레임 이상 관측'을 요구한다. 매 프레임 잡히는
실제 장애물은 M 프레임 만에 통과하지만, 띄엄띄엄 잡히는 노이즈는 창이
흘러가면서 영원히 통과하지 못한다. 시간 누적과 달리 창 밖으로 나간 과거
관측은 버려지기 때문이다.

승격은 엄격하게, 강등은 하지 않는다. 한 번 확정된 트랙은 회피 도중 잠깐
가려지거나 클러스터 임계에 걸려 놓쳐도 확정을 유지해야 안전하다. 트랙 자체의
수명은 호출부의 keep_time_s / track_keep_time_s 가 관리한다.
"""

from __future__ import annotations


def _popcount(x: int) -> int:
    try:
        return x.bit_count()          # Python 3.10+
    except AttributeError:            # pragma: no cover
        return bin(x).count("1")


class HitHistory:
    """최근 window 프레임의 관측 여부를 비트로 들고 M-of-N 을 판정한다.

    비트마스크라 프레임마다 할당이 없고, 40Hz x 트랙 수 정도는 비용이
    사실상 0이다.
    """

    __slots__ = ("_bits", "_mask", "_min_hits", "_confirmed")

    def __init__(self, window: int, min_hits: int, *, initial_hit: bool = True):
        window = max(1, int(window))
        # min_hits 1 이면 예전 동작(첫 관측에 바로 확정)과 같아진다.
        self._min_hits = max(1, min(int(min_hits), window))
        self._mask = (1 << window) - 1
        self._bits = 0
        self._confirmed = False
        self.update(initial_hit)

    def update(self, hit: bool) -> bool:
        """한 프레임 진행. 확정 상태를 돌려준다."""
        self._bits = ((self._bits << 1) | (1 if hit else 0)) & self._mask
        if not self._confirmed and _popcount(self._bits) >= self._min_hits:
            self._confirmed = True
        return self._confirmed

    @property
    def confirmed(self) -> bool:
        return self._confirmed

    @property
    def hits(self) -> int:
        """현재 창 안의 관측 횟수. 디버그/로깅용."""
        return _popcount(self._bits)
