"""시각화 전용 토픽을 "듣는 데가 있을 때만" 내보내기 위한 게이트.

rclpy 는 구독자가 0 이어도 메시지를 끝까지 직렬화한다. 파이썬 객체를 C
구조체로 옮기는 비용이 그대로 들어간다는 뜻이다. 20260822 실측 (Orin, 이
스택이 돌아가는 상태):

    Float64   구독자 0 발행      11 µs
    Path(140) 구독자 0 발행    4341 µs
    get_subscription_count()    1.8 µs

Foxglove 를 닫아 둔 채(=레이스 중) `/waypoint_tracked_path` 를 33 Hz 로 내면
코어의 14 % 가 아무도 안 보는 직렬화에 들어간다. 검사 한 번이 발행의
1/2400 이라 게이트를 다는 쪽이 언제나 싸다.

**제어 경로 토픽에는 쓰지 말 것.** 구독자가 붙기 직전에 발행을 건너뛰면 그
한 장을 잃는데, 게이트/모드처럼 상태를 나르는 토픽은 그 한 장이 늦으면
거동이 달라진다. 여기 쓰는 대상은 소비자가 Foxglove 뿐인 토픽으로 한정한다
(`debug/topic_consumers.py` 로 확인할 수 있다).
"""
from __future__ import annotations


def has_listener(pub) -> bool:
    """이 퍼블리셔를 듣는 구독자가 하나라도 있나."""
    if pub is None:
        return False
    try:
        return pub.get_subscription_count() > 0
    except Exception:
        # 조회가 안 되면 기존처럼 그냥 낸다 — 시각화가 조용히 사라지는 것보다 낫다.
        return True
