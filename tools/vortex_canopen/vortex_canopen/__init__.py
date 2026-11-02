"""vortex-canopen: host tooling for the HardFOC Vortex V1 CANopen device.

`od_ids` and `data/vortex_v1.eds` come from od/vortex_od.yaml (tools/odgen).
"""

from importlib import resources


def eds_path() -> str:
    """Path of the packaged EDS that matches this tool version."""
    return str(resources.files(__name__) / "data" / "vortex_v1.eds")
