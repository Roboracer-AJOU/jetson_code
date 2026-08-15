#!/usr/bin/env python3
"""등속(constant-velocity) 칼만 필터. 장애물 트랙 속도 추정용.

유한차분 + EMA 는 dt 지터와 부분 가림에 약하다. 스캔 한 프레임에서 클러스터
경계가 한 점만 달라져도 중심이 튀고, 그게 1/dt 로 증폭돼 속도로 들어간다.
EMA 는 그걸 늦출 뿐 없애지 못한다 (그리고 늦추는 만큼 진짜 가속도 늦는다).

상태 x = [px, py, vx, vy], 관측은 위치만. 외부 의존성 없이 numpy 4x4 로만
푼다 — 트랙 몇 개 수준에서는 행렬 크기가 작아 비용이 무시할 만하다.
"""
from __future__ import annotations

import math

import numpy as np


class ConstantVelocityKF:
    """위치 관측 등속 KF.

    sigma_accel 은 "이 물체가 얼마나 급하게 방향을 바꿀 수 있나" 다.
    크게 잡으면 반응이 빠른 대신 노이즈를 그대로 먹는다.
    """

    __slots__ = ("x", "P", "_sa2", "_sm2")

    def __init__(
        self,
        px: float,
        py: float,
        sigma_accel: float = 3.0,
        sigma_meas: float = 0.06,
        vx: float = 0.0,
        vy: float = 0.0,
    ) -> None:
        self.x = np.array([px, py, vx, vy], dtype=np.float64)
        self._sa2 = float(sigma_accel) ** 2
        self._sm2 = float(sigma_meas) ** 2
        # 초기 속도는 모른다 → 속도 분산을 크게 열어 첫 관측에서 빨리 잡히게
        self.P = np.diag([self._sm2, self._sm2, 25.0, 25.0]).astype(np.float64)

    @property
    def vx(self) -> float:
        return float(self.x[2])

    @property
    def vy(self) -> float:
        return float(self.x[3])

    @property
    def px(self) -> float:
        return float(self.x[0])

    @property
    def py(self) -> float:
        return float(self.x[1])

    @property
    def speed(self) -> float:
        return math.hypot(float(self.x[2]), float(self.x[3]))

    def predict(self, dt: float) -> None:
        if dt <= 0.0:
            return
        x = self.x
        # F @ x 를 직접 전개 (4x4 곱보다 싸다)
        x[0] += x[2] * dt
        x[1] += x[3] * dt

        P = self.P
        # P = F P Fᵀ + Q, 등속 모델의 이산 백색가속 잡음
        d2 = dt * dt
        d3 = d2 * dt
        d4 = d3 * dt
        q = self._sa2
        # F P Fᵀ
        P[0, 0] += dt * (P[2, 0] + P[0, 2]) + d2 * P[2, 2]
        P[0, 1] += dt * (P[2, 1] + P[0, 3]) + d2 * P[2, 3]
        P[1, 0] += dt * (P[3, 0] + P[1, 2]) + d2 * P[3, 2]
        P[1, 1] += dt * (P[3, 1] + P[1, 3]) + d2 * P[3, 3]
        P[0, 2] += dt * P[2, 2]
        P[0, 3] += dt * P[2, 3]
        P[1, 2] += dt * P[3, 2]
        P[1, 3] += dt * P[3, 3]
        P[2, 0] += dt * P[2, 2]
        P[3, 0] += dt * P[3, 2]
        P[2, 1] += dt * P[2, 3]
        P[3, 1] += dt * P[3, 3]
        # + Q
        P[0, 0] += 0.25 * d4 * q
        P[1, 1] += 0.25 * d4 * q
        P[0, 2] += 0.5 * d3 * q
        P[2, 0] += 0.5 * d3 * q
        P[1, 3] += 0.5 * d3 * q
        P[3, 1] += 0.5 * d3 * q
        P[2, 2] += d2 * q
        P[3, 3] += d2 * q

    def mahalanobis2(self, zx: float, zy: float) -> float:
        """관측 잔차의 마할라노비스 거리 제곱. 게이팅용."""
        S = self.P[:2, :2] + np.eye(2) * self._sm2
        try:
            Sinv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return float("inf")
        y = np.array([zx - self.x[0], zy - self.x[1]])
        return float(y @ Sinv @ y)

    def update(self, zx: float, zy: float) -> None:
        P = self.P
        S = P[:2, :2] + np.eye(2) * self._sm2
        try:
            Sinv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return
        K = P[:, :2] @ Sinv                       # 4x2
        y = np.array([zx - self.x[0], zy - self.x[1]])
        self.x += K @ y
        # Joseph 형태까지는 안 간다 — (I-KH)P 로 충분하고 P 가 작다
        self.P = P - K @ P[:2, :]
        # 수치 대칭성 유지
        self.P = 0.5 * (self.P + self.P.T)
