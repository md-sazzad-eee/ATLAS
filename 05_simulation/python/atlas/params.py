"""Load the shared parameter file (single source of truth)."""
import json
import os
import pathlib

# ATLAS repo root: env var (set by the ROS 2 launch files, where this package is installed
# elsewhere) or the source-tree location.
ROOT = pathlib.Path(os.environ.get("ATLAS_ROOT", pathlib.Path(__file__).resolve().parents[3]))
PARAMS_JSON = ROOT / "00_requirements" / "atlas_params.json"


def load_params(path: pathlib.Path = PARAMS_JSON) -> dict:
    return json.loads(pathlib.Path(path).read_text())
