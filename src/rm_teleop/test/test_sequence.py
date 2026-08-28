from rm_teleop.receiver_core import SequenceStatus, SequenceTracker


def test_normal_sequence() -> None:
    tracker = SequenceTracker()

    assert tracker.classify(100).status == SequenceStatus.FIRST
    assert tracker.classify(101).status == SequenceStatus.OK


def test_duplicate_is_rejected() -> None:
    tracker = SequenceTracker()
    tracker.classify(100)

    decision = tracker.classify(100)

    assert decision.accepted is False
    assert decision.status == SequenceStatus.DUPLICATE


def test_out_of_order_is_rejected_without_advancing_state() -> None:
    tracker = SequenceTracker()
    tracker.classify(100)
    tracker.classify(101)

    old = tracker.classify(99)
    next_packet = tracker.classify(102)

    assert old.accepted is False
    assert old.status == SequenceStatus.OUT_OF_ORDER
    assert next_packet.accepted is True
    assert next_packet.status == SequenceStatus.OK


def test_gap_counts_dropped_packets() -> None:
    tracker = SequenceTracker()
    tracker.classify(100)

    decision = tracker.classify(104)

    assert decision.accepted is True
    assert decision.status == SequenceStatus.DROP
    assert decision.dropped == 3


def test_uint32_wrap_is_accepted() -> None:
    tracker = SequenceTracker()
    tracker.classify(0xFFFFFFFF)

    decision = tracker.classify(0)

    assert decision.accepted is True
    assert decision.status == SequenceStatus.OK
