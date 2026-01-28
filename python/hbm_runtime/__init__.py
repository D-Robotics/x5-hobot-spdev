# flake8: noqa: E402
import os
import ctypes


_REQUIRED_LIBS = {
    # "libhbucp.so": "Horizon UCP library",
    "libdnn.so": "Horizon DNN library",
}


def _check_system_libs():
    if os.environ.get("HBM_RUNTIME_SKIP_CHECK", "0") == "1":
        return

    flags = getattr(ctypes, "RTLD_LOCAL", 0) | getattr(ctypes, "RTLD_NOW", 2)
    missing = []
    for soname, desc in _REQUIRED_LIBS.items():
        try:
            ctypes.CDLL(soname, mode=flags)
        except OSError as e:
            missing.append(f"{soname} ({desc}) - {e}")

    if missing:
        raise RuntimeError(
            "Missing Horizon runtime libraries (provided by the 'hobot-dnn' package):\n  - "
            + "\n  - ".join(missing)
            + "\n\nHow to fix:\n"
            "  1) Install or upgrade the 'hobot-dnn' deb package:\n"
            "     sudo apt-get update && sudo apt-get install -y hobot-dnn\n"
            "  2) Set HBM_RUNTIME_SKIP_CHECK=1 to skip this check.\n"
        )


_check_system_libs()

from .HB_HBMRuntime import (
    HB_HBMRuntime,
    hbDNNDataType,
    hbDNNQuantiType,
    QuantParams,
    SchedParam
)

__all__ = [
    "HB_HBMRuntime",
    "hbDNNDataType",
    "hbDNNQuantiType",
    "QuantParams",
    "SchedParam"
]
