from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from .protocol import ProtocolConfig, ProtocolError, TeleopPacket, parse_datagram

UINT32_MODULUS = 1 << 32
UINT32_HALF_RANGE = 1 << 31


class SequenceStatus(str, Enum):
    FIRST = "first"
    OK = "ok"
    DROP = "drop"
    DUPLICATE = "duplicate"
    OUT_OF_ORDER = "out_of_order"
    RESET_AFTER_GAP = "reset_after_gap"
    SOURCE_RESTART = "source_restart"


@dataclass(frozen=True)
class SequenceDecision:
    accepted: bool
    status: SequenceStatus
    dropped: int = 0


class SequenceTracker:
    def __init__(self) -> None:
        self.last_seq: Optional[int] = None

    def reset(self) -> None:
        self.last_seq = None

    def classify(self, seq: int) -> SequenceDecision:
        if self.last_seq is None:
            self.last_seq = seq
            return SequenceDecision(True, SequenceStatus.FIRST)

        delta = (seq - self.last_seq) % UINT32_MODULUS

        if delta == 0:
            return SequenceDecision(False, SequenceStatus.DUPLICATE)

        if delta < UINT32_HALF_RANGE:
            dropped = delta - 1
            self.last_seq = seq
            if dropped:
                return SequenceDecision(True, SequenceStatus.DROP, dropped=dropped)
            return SequenceDecision(True, SequenceStatus.OK)

        return SequenceDecision(False, SequenceStatus.OUT_OF_ORDER)


@dataclass
class ReceiverCounters:
    datagrams: int = 0
    accepted: int = 0
    invalid: int = 0
    duplicates: int = 0
    out_of_order: int = 0
    dropped: int = 0
    sequence_resets: int = 0
    source_restarts: int = 0
    timeouts: int = 0
    invalid_reasons: Counter[str] = field(default_factory=Counter)


@dataclass(frozen=True)
class ProcessResult:
    accepted: bool
    packet: Optional[TeleopPacket]
    reason: str
    sequence_status: Optional[SequenceStatus] = None
    dropped: int = 0


class ReceiverCore:
    def __init__(
        self,
        protocol_config: ProtocolConfig | None = None,
        sequence_reset_after_sec: float = 1.0,
        source_restart_rollback_sec: float = 0.5,
    ) -> None:
        if sequence_reset_after_sec <= 0.0:
            raise ValueError("sequence_reset_after_sec must be positive")
        if source_restart_rollback_sec <= 0.0:
            raise ValueError("source_restart_rollback_sec must be positive")

        self.protocol_config = protocol_config or ProtocolConfig()
        self.sequence_reset_after_sec = sequence_reset_after_sec
        self.source_restart_rollback_sec = source_restart_rollback_sec

        self.sequence = SequenceTracker()
        self.counters = ReceiverCounters()

        self.last_datagram_monotonic: Optional[float] = None
        self.last_accepted_monotonic: Optional[float] = None
        self.last_valid_palm_monotonic: Optional[float] = None
        self.last_source_timestamp_sec: Optional[float] = None
        self.last_accepted_packet: Optional[TeleopPacket] = None
        self._timeout_latched = False

    def process_datagram(self, data: bytes, now_monotonic: float) -> ProcessResult:
        self.counters.datagrams += 1
        self.last_datagram_monotonic = now_monotonic

        try:
            packet = parse_datagram(data, self.protocol_config)
        except ProtocolError as exc:
            self.counters.invalid += 1
            self.counters.invalid_reasons[exc.code] += 1
            return ProcessResult(False, None, f"protocol:{exc.code}")

        reset_after_gap = False
        if (
            self.last_accepted_monotonic is not None
            and now_monotonic - self.last_accepted_monotonic
            >= self.sequence_reset_after_sec
        ):
            self.sequence.reset()
            self.counters.sequence_resets += 1
            reset_after_gap = True

        decision = self.sequence.classify(packet.seq)

        if not decision.accepted and decision.status == SequenceStatus.OUT_OF_ORDER:
            source_restarted = (
                self.last_source_timestamp_sec is not None
                and packet.source_timestamp_sec
                < self.last_source_timestamp_sec - self.source_restart_rollback_sec
            )
            if source_restarted:
                self.sequence.reset()
                self.sequence.classify(packet.seq)
                self.counters.source_restarts += 1
                decision = SequenceDecision(True, SequenceStatus.SOURCE_RESTART)

        if not decision.accepted:
            if decision.status == SequenceStatus.DUPLICATE:
                self.counters.duplicates += 1
            else:
                self.counters.out_of_order += 1
            return ProcessResult(
                False,
                None,
                f"sequence:{decision.status.value}",
                sequence_status=decision.status,
            )

        status = decision.status
        if reset_after_gap and status == SequenceStatus.FIRST:
            status = SequenceStatus.RESET_AFTER_GAP

        self.counters.accepted += 1
        self.counters.dropped += decision.dropped
        self.last_accepted_monotonic = now_monotonic
        self.last_source_timestamp_sec = packet.source_timestamp_sec
        self.last_accepted_packet = packet
        if packet.palm.valid:
            self.last_valid_palm_monotonic = now_monotonic
        self._timeout_latched = False

        return ProcessResult(
            True,
            packet,
            "accepted",
            sequence_status=status,
            dropped=decision.dropped,
        )

    def accepted_age_sec(self, now_monotonic: float) -> Optional[float]:
        if self.last_accepted_monotonic is None:
            return None
        return max(0.0, now_monotonic - self.last_accepted_monotonic)

    def valid_palm_age_sec(self, now_monotonic: float) -> Optional[float]:
        if self.last_valid_palm_monotonic is None:
            return None
        return max(0.0, now_monotonic - self.last_valid_palm_monotonic)

    def is_timed_out(self, now_monotonic: float, timeout_sec: float) -> bool:
        if timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be positive")
        age = self.accepted_age_sec(now_monotonic)
        return age is None or age > timeout_sec

    def latch_timeout_if_new(self, now_monotonic: float, timeout_sec: float) -> bool:
        timed_out = self.is_timed_out(now_monotonic, timeout_sec)
        if timed_out and not self._timeout_latched:
            self._timeout_latched = True
            self.counters.timeouts += 1
            return True
        if not timed_out:
            self._timeout_latched = False
        return False
