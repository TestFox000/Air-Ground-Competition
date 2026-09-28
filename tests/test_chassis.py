import pytest

from ag_chassis.chassis_link import ChassisLink, DriveTask


def _task(cell="C3", eta=4.0, **kw) -> DriveTask:
    return DriveTask(type=kw.get("type", "mine"), cell=cell,
                     path=kw.get("path", ["B2", "B3", "C3"]), eta_s=eta)


def test_start_and_progress():
    link = ChassisLink(time_scale=1.0)
    assert link.start(_task()) is True
    assert link.busy
    assert link.tick(1.0) is None
    assert link.state.progress == pytest.approx(0.25)


def test_arrival_returns_cell():
    link = ChassisLink(time_scale=1.0)
    link.start(_task(cell="C3", eta=2.0))
    assert link.tick(1.0) is None
    assert link.tick(1.0) == "C3"
    assert link.busy is False
    assert link.state.status == "arrived"


def test_time_scale_fast_forward():
    link = ChassisLink(time_scale=10.0)
    link.start(_task(cell="K14", eta=8.0))
    assert link.tick(0.1) is None    # 累计 1 s，未到 8 s
    assert link.tick(0.7) == "K14"   # 再累计 7 s，共 8 s -> 到位


def test_ignores_new_task_while_moving():
    link = ChassisLink()
    link.start(_task(cell="C3"))
    assert link.start(_task(cell="D4")) is False
    assert link.state.cell == "C3"


def test_accepts_new_task_after_arrival():
    link = ChassisLink(time_scale=100.0)
    link.start(_task(cell="C3"))
    link.tick(0.1)
    assert link.start(_task(cell="D4")) is True
    assert link.state.cell == "D4"


def test_abort():
    link = ChassisLink()
    link.start(_task())
    link.abort("stop")
    assert link.busy is False
    assert link.state.status == "aborted"
    assert link.tick(10.0) is None


def test_tick_without_task_is_noop():
    link = ChassisLink()
    assert link.tick(1.0) is None
    assert link.state.status == "idle"


def test_zero_eta_completes_immediately():
    link = ChassisLink()
    link.start(_task(cell="A1", eta=0.0))
    assert link.tick(0.0) == "A1"


def test_real_mode_falls_back_to_timer_when_protocol_missing():
    """实机模式且协议未对接时，_send_path 会抛 NotImplementedError。"""
    link = ChassisLink(simulate=False)
    with pytest.raises(NotImplementedError):
        link.start(_task())
