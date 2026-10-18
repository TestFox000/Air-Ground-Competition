"""patch_deploy 回滚/备份命令串的结构测试（零硬件：只验证拼出的远程命令形态）。

覆盖审查实锤的两条回滚坑：
  * 假阳性：旧命令串用 `;` 分隔，备份缺失/cp 失败/编译没跑都照打
    ROLLBACK_DONE——现在必须全程 && 串联，且编译产物不许再过管道
    （管道会吞掉 colcon 的退出码）；
  * 选错备份：旧实现 `ls -t *.bak_* | head -1` 会混进手工 .bak_local_*、
    还可能选到"已是补丁版"的二次备份——现在只认部署时写入的回滚索引，
    且回滚前必须 md5 指纹校验，索引缺失/不符要明确拒绝（ROLLBACK_REFUSED）。
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

pytest.importorskip("paramiko")   # patch_deploy 顶层经 remote_run import paramiko

REPO = Path(__file__).resolve().parents[1]


def load_module(name: str, file: Path):
    spec = importlib.util.spec_from_file_location(name, file)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


pd = load_module("patch_deploy", REPO / "tools" / "patch_deploy.py")


# ---------------------------------------------------------------- 回滚命令（审查#3/#4）

def test_rollback_cmd_uses_index_not_ls_t():
    cmd = pd.rollback_cmd()
    assert pd.REMOTE_INDEX.strip("~").lstrip("/") in cmd or ".patch_deploy_index" in cmd
    assert "ls -t" not in cmd                    # 不再按时间猜最新备份
    assert "*.bak_*" not in cmd                  # 不再裸匹配别人的 .bak_local_*


def test_rollback_cmd_validates_backup_explicitly():
    cmd = pd.rollback_cmd()
    assert '[ -n "$CPP_BAK" ]' in cmd and '[ -f "$CPP_BAK" ]' in cmd
    assert '[ -n "$HPP_BAK" ]' in cmd and '[ -f "$HPP_BAK" ]' in cmd
    assert "md5sum" in cmd                       # 指纹校验（diff 口径）
    assert cmd.count("ROLLBACK_REFUSED") >= 3    # 索引缺失/备份缺失/指纹不符都拒


def test_rollback_cmd_fails_loudly_without_false_positive():
    cmd = pd.rollback_cmd()
    # 全程 && 串联到链尾才打 ROLLBACK_DONE：中间任一环节失败链条即断
    assert " && echo ROLLBACK_DONE" in cmd
    assert "echo ROLLBACK_DONE;" not in cmd
    # 编译结果不许过管道（| tail 会吞掉 colcon 退出码造成假阳性）
    assert "colcon build --packages-select px4_odom_convert | " not in cmd
    assert 'cp "$CPP_BAK"' in cmd and 'cp "$HPP_BAK"' in cmd


# ---------------------------------------------------------------- 备份命令（审查#4 的写入侧）

def test_backup_cmd_writes_index_with_fingerprints():
    cmd = pd.backup_cmd("20261017_120000")
    assert ".bak_patch_20261017_120000" in cmd   # 只产 .bak_patch_* 形态备份
    assert ".patch_deploy_index" in cmd          # 写回滚索引
    assert 'CPP_MD5=$(md5sum' in cmd and 'HPP_MD5=$(md5sum' in cmd
    assert "&& echo BACKUP_OK" in cmd            # 链尾标志（cp 失败不会假阳性）


# ---------------------------------------------------------------- step() 的 refuse 语义（审查#3/#4 配套）

def test_step_refuse_aborts_immediately_without_retry(monkeypatch, capsys):
    """对端确定性拒绝（ROLLBACK_REFUSED）：原样亮证据，一次都不重试。"""
    calls = []

    def fake_run(*a, **kw):
        calls.append(1)
        return 1, "ROLLBACK_REFUSED: 无回滚索引\n索引证据 xx", ""

    monkeypatch.setattr(pd.remote_run, "run", fake_run)
    with pytest.raises(SystemExit):
        pd.step("h", "u", "p", 22, "回滚", "cmd",
                expect="ROLLBACK_DONE", refuse="ROLLBACK_REFUSED")
    assert len(calls) == 1
    assert "ROLLBACK_REFUSED" in capsys.readouterr().out   # 证据必须打出来


def test_step_retries_on_missing_expect_then_fails(monkeypatch):
    """expect 不中（非确定性失败，如断链/节点没起完）才走 5 次重试。"""
    monkeypatch.setattr(pd.time, "sleep", lambda _s: None)
    calls = []

    def fake_run(*a, **kw):
        calls.append(1)
        return 0, "什么都没有", ""

    monkeypatch.setattr(pd.remote_run, "run", fake_run)
    with pytest.raises(SystemExit):
        pd.step("h", "u", "p", 22, "验证", "cmd", expect="PROC_OK")
    assert len(calls) == 5
