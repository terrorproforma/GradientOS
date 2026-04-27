# robots/fr10/__init__.py
#
# RobotConfig for the Fairino FR10 6-DOF industrial arm. Used together with
# the Fairino ActuatorBackend (backends/fairino/) which talks to the FR10
# controller over its Python SDK.

from .config import FR10Config

__all__ = ["FR10Config"]
