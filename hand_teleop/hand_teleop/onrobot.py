#!/usr/bin/env python3
"""OnRobot RG2용 최소 Modbus TCP 드라이버.

이 파일은 ``hand_follow_robot_node_ver_depth``가 필요한 기능만 제공합니다.
기존 프로젝트의 raw API는 폭과 힘을 각각 0.1 mm, 0.1 N 단위로 받기 때문에
800을 800 mm로 오해하기 쉽습니다. 이 드라이버의 공개 이동 함수는 mm와 N을
받고 내부에서 raw 정수로 변환합니다.
"""

from __future__ import annotations

from dataclasses import dataclass

from pymodbus.client.sync import ModbusTcpClient


@dataclass(frozen=True)
class RG2Status:
    """RG2 상태 레지스터에서 사용하는 일곱 개의 상태 비트."""

    busy: bool
    grip_detected: bool
    safety_switch_1_pushed: bool
    safety_circuit_1_triggered: bool
    safety_switch_2_pushed: bool
    safety_circuit_2_triggered: bool
    safety_error: bool

    @property
    def safety_fault(self) -> bool:
        """동작을 계속하면 안 되는 안전 회로 관련 상태인지 반환합니다."""

        return (
            self.safety_switch_1_pushed
            or self.safety_circuit_1_triggered
            or self.safety_switch_2_pushed
            or self.safety_circuit_2_triggered
            or self.safety_error
        )


class RG2:
    """RG2 한 대와 통신하는 Modbus TCP 연결입니다."""

    # OnRobot RG 계열의 Modbus slave/unit ID입니다.
    UNIT_ID = 65

    # RG2 공식 동작 범위입니다.
    MIN_WIDTH_MM = 0.0
    MAX_WIDTH_MM = 110.0
    MIN_FORCE_N = 3.0
    MAX_FORCE_N = 40.0

    # RG2 Modbus 값은 실제 단위의 10배인 정수로 전달합니다.
    WIDTH_RAW_PER_MM = 10.0
    FORCE_RAW_PER_N = 10.0

    def __init__(
        self,
        ip: str,
        port: int = 502,
        timeout_sec: float = 1.0,
    ) -> None:
        """지정한 IP와 포트로 RG2 Modbus TCP 연결을 엽니다."""

        self._client = ModbusTcpClient(
            host=ip,
            port=port,
            timeout=timeout_sec,
        )
        if not self._client.connect():
            raise ConnectionError(
                f"RG2 Modbus TCP 연결에 실패했습니다: {ip}:{port}"
            )

    @staticmethod
    def _raise_on_modbus_error(response, operation: str) -> None:
        """pymodbus 오류 응답을 읽기 쉬운 예외로 변환합니다."""

        if response is None:
            raise ConnectionError(f"RG2 {operation} 응답이 없습니다.")
        if hasattr(response, "isError") and response.isError():
            raise IOError(f"RG2 {operation} 오류 응답: {response}")

    @classmethod
    def _width_to_raw(cls, width_mm: float) -> int:
        """폭 [mm]을 0.1 mm 단위 Modbus 정수로 변환합니다."""

        if not cls.MIN_WIDTH_MM <= width_mm <= cls.MAX_WIDTH_MM:
            raise ValueError(
                "RG2 폭은 "
                f"{cls.MIN_WIDTH_MM:.1f}~{cls.MAX_WIDTH_MM:.1f} mm여야 합니다."
            )
        return int(round(width_mm * cls.WIDTH_RAW_PER_MM))

    @classmethod
    def _force_to_raw(cls, force_n: float) -> int:
        """힘 [N]을 0.1 N 단위 Modbus 정수로 변환합니다."""

        if not cls.MIN_FORCE_N <= force_n <= cls.MAX_FORCE_N:
            raise ValueError(
                "RG2 힘은 "
                f"{cls.MIN_FORCE_N:.1f}~{cls.MAX_FORCE_N:.1f} N이어야 합니다."
            )
        return int(round(force_n * cls.FORCE_RAW_PER_N))

    def get_status(self) -> RG2Status:
        """상태 레지스터 268을 읽어 의미 있는 bool 값으로 반환합니다."""

        response = self._client.read_holding_registers(
            address=268,
            count=1,
            unit=self.UNIT_ID,
        )
        self._raise_on_modbus_error(response, "상태 읽기")

        if not hasattr(response, "registers") or not response.registers:
            raise IOError("RG2 상태 응답에 레지스터 값이 없습니다.")
        raw_status = int(response.registers[0])

        return RG2Status(
            busy=bool(raw_status & (1 << 0)),
            grip_detected=bool(raw_status & (1 << 1)),
            safety_switch_1_pushed=bool(raw_status & (1 << 2)),
            safety_circuit_1_triggered=bool(raw_status & (1 << 3)),
            safety_switch_2_pushed=bool(raw_status & (1 << 4)),
            safety_circuit_2_triggered=bool(raw_status & (1 << 5)),
            safety_error=bool(raw_status & (1 << 6)),
        )

    def move_gripper_mm_n(
        self,
        width_mm: float,
        force_n: float,
    ) -> None:
        """지정한 폭 [mm]과 힘 [N]으로 RG2 이동을 한 번 시작합니다."""

        width_raw = self._width_to_raw(width_mm)
        force_raw = self._force_to_raw(force_n)

        # 레지스터 0: 목표 힘, 1: 목표 폭, 2: 제어 명령입니다.
        # control=16은 fingertip offset을 적용해 이동하라는 뜻입니다.
        response = self._client.write_registers(
            address=0,
            values=[force_raw, width_raw, 16],
            unit=self.UNIT_ID,
        )
        self._raise_on_modbus_error(response, "이동 명령 쓰기")

    def close_connection(self) -> None:
        """열린 Modbus TCP 소켓을 닫습니다."""

        self._client.close()
