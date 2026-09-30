"""Torch-free loader for LIBERO init-state files.

LIBERO's ``Benchmark.get_task_init_states`` calls ``torch.load`` on
``<task>.pruned_init``.  Those files are zip-format torch pickles whose payload is a
plain ``numpy.ndarray`` (N, 1 + nq + nv) of flattened MjSimState rows
``[time, qpos, qvel]``.  With torch >= 2.6 the default ``weights_only=True`` refuses
them.  We load the pickle directly; any reference to a torch class raises so we can
never silently accept a file that is not a pure numpy payload.
"""
from __future__ import annotations

import io
import pickle
import zipfile

import numpy as np


class _NumpyOnlyUnpickler(pickle.Unpickler):
    def persistent_load(self, pid):  # torch storages would arrive here
        raise RuntimeError(f"unexpected torch storage in init file: {pid!r}")

    def find_class(self, module, name):
        if module.startswith("torch"):
            raise RuntimeError(f"refusing to unpickle torch class {module}.{name}")
        return super().find_class(module, name)


def load_init_states(path: str) -> np.ndarray:
    """Return the (N, D) float64 array of official initial states stored in *path*."""
    with zipfile.ZipFile(path) as zf:
        pkl_names = [n for n in zf.namelist() if n.endswith("data.pkl")]
        if len(pkl_names) != 1:
            raise ValueError(f"{path}: expected exactly one data.pkl, found {pkl_names}")
        arr = _NumpyOnlyUnpickler(io.BytesIO(zf.read(pkl_names[0]))).load()
    arr = np.asarray(arr, dtype=np.float64)
    if arr.ndim != 2:
        raise ValueError(f"{path}: expected 2-D array, got shape {arr.shape}")
    return arr
