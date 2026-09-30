import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
for p in [os.path.join(ROOT, "src"), os.path.join(ROOT, "build", "intrinsic_py")]:
    if p not in sys.path:
        sys.path.insert(0, p)
os.environ.setdefault("MUJOCO_GL", "egl")
