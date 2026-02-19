# backends/simulation/config.py
#
# Simulation backend configuration constants.
#
# Simulation reuses Feetech-compatible constants/parsers so existing utility and
# telemetry plumbing can operate unchanged while still reporting the backend as
# "simulation" in runtime logs/state.

from ..feetech.config import *  # noqa: F401,F403

