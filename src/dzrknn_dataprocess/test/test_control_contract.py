#!/usr/bin/env python3
"""Compile the real gimbal callbacks/header with a deterministic ROS-free fixture.

Run with Python 3 and g++ (Windows also supports g++ in WSL Ubuntu). This tests
production decisions and emitted messages, not DDS, node startup, or hardware.
Only middleware includes and the steady clock implementation are substituted;
the production callback/control bodies and field declarations remain unchanged.
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--suite", choices=("home", "gate", "all"), default="all")
    args = parser.parse_args()
    source = (args.source_root / "src/dzrknn_dataprocess.cpp").read_text(encoding="utf-8-sig")
    header = (args.source_root / "include/dzrknn_dataprocess.h").read_text(encoding="utf-8-sig")
    # This contiguous block includes the complete vision/fire callback and its
    # math, scanning, feedback, and permission callbacks. No decision is copied
    # into the fixture. Calibration I/O, ROS startup and the hardware loop are out.
    begin = source.index("void turn_on_robot::callback_offset_center(")
    end = source.index("int turn_on_robot::compute_comp_pixel_0(", begin)
    source = source[begin:end]
    header = re.sub(r'^\s*#include\s*[<"](?:rclcpp/|boost/|tf2_ros/|std_msgs/|sensor_msgs/).*$',
                    "", header, flags=re.MULTILINE)
    source = source.replace("std::chrono::steady_clock", "FixtureSteadyClock")
    header = header.replace("std::chrono::steady_clock", "FixtureSteadyClock")
    fixture = Path(__file__).with_name("control_contract_fixture.cpp").read_text(encoding="utf-8")
    with tempfile.TemporaryDirectory(prefix="gimbal-contract-") as temp:
        work = Path(temp)
        (work / "production_header.h").write_text(header, encoding="utf-8")
        (work / "production_callbacks.cpp").write_text(source, encoding="utf-8")
        test_cpp = work / "control_contract_fixture.cpp"
        test_cpp.write_text(fixture, encoding="utf-8")
        executable = work / ("contract.exe" if os.name == "nt" else "contract")
        compiler = shutil.which("g++")
        if compiler:
            compile_cmd = [compiler, "-std=c++14", "-Wall", "-Wextra", "-pedantic", str(test_cpp), "-o", str(executable)]
            run_cmd = [str(executable), args.suite]
        elif os.name == "nt" and shutil.which("wsl"):
            def linux_path(path: Path) -> str:
                return subprocess.check_output(
                    ["wsl", "-d", "Ubuntu", "--", "wslpath", "-a", path.as_posix()],
                    encoding="utf-8").strip()
            linux_cpp, linux_exe = linux_path(test_cpp), linux_path(executable)
            compile_cmd = ["wsl", "-d", "Ubuntu", "--", "g++", "-std=c++14", "-Wall", "-Wextra", "-pedantic", linux_cpp, "-o", linux_exe]
            run_cmd = ["wsl", "-d", "Ubuntu", "--", linux_exe, args.suite]
        else:
            parser.error("g++ is required (or install it in WSL Ubuntu on Windows)")
        print(f"Compile real callbacks: {args.source_root.resolve()}", flush=True)
        result = subprocess.run(compile_cmd)
        if result.returncode:
            return result.returncode
        return subprocess.run(run_cmd).returncode


if __name__ == "__main__":
    raise SystemExit(main())
