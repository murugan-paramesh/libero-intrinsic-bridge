#!/usr/bin/env python
"""Write ~/.libero/config.yaml non-interactively (LIBERO prompts on first import otherwise)."""
import os, sys, yaml
root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "third_party", "LIBERO", "libero", "libero"))
cfg_dir = os.environ.get("LIBERO_CONFIG_PATH", os.path.expanduser("~/.libero"))
os.makedirs(cfg_dir, exist_ok=True)
cfg = {"benchmark_root": root, "bddl_files": root + "/bddl_files", "init_states": root + "/init_files",
       "datasets": os.path.abspath(os.path.join(root, "..", "..", "datasets")), "assets": root + "/assets"}
with open(os.path.join(cfg_dir, "config.yaml"), "w") as f:
    yaml.dump(cfg, f)
print(cfg)
