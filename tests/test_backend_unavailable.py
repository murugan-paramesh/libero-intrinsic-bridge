"""Integration check: an unavailable Intrinsic backend must raise an explicit error; there is no
silent fallback to a local planner anywhere in the code base."""
import os
import subprocess

import pytest

from libero_intrinsic.intrinsic.client import IntrinsicClient, IntrinsicServer, IntrinsicUnavailableError, RequestLog, ServerAddresses


def test_missing_binary_raises(tmp_path):
    s = IntrinsicServer("/nonexistent/libero_planner_server", {"w": "x.sdf"}, str(tmp_path))
    with pytest.raises(IntrinsicUnavailableError):
        s.start()


def test_crashing_binary_raises(tmp_path):
    fake = tmp_path / "fake_server.sh"
    fake.write_text("#!/bin/sh\necho 'boom' >&2\nexit 3\n")
    fake.chmod(0o755)
    s = IntrinsicServer(str(fake), {"w": "x.sdf"}, str(tmp_path), timeout_s=5)
    with pytest.raises(IntrinsicUnavailableError) as ei:
        s.start()
    assert "exited with code 3" in str(ei.value)


def test_unreachable_service_raises(tmp_path):
    addrs = ServerAddresses(world_service="127.0.0.1:1", motion_planner_service="127.0.0.1:1", credentials="local_tcp")
    log = RequestLog(str(tmp_path / "log.jsonl"))
    with pytest.raises(IntrinsicUnavailableError):
        IntrinsicClient(addrs, "w", log)


def test_no_fallback_planner_in_codebase():
    """Guard: the only planning entry points are Intrinsic RPCs."""
    root = os.path.join(os.path.dirname(__file__), "..", "src", "libero_intrinsic")
    src = "\n".join(open(os.path.join(dp, f)).read() for dp, _, fs in os.walk(root) for f in fs if f.endswith(".py"))
    for banned in ["import ompl", "from ompl", "import klampt", "import pinocchio", "import roboticstoolbox", "import ikpy", "import pybullet"]:
        assert banned not in src
