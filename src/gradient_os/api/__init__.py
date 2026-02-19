"""GradientOS HTTP API package.

Keep this module side-effect free so running ``python -m gradient_os.api.main``
does not pre-import ``gradient_os.api.main`` during package import, which would
otherwise trigger a runpy re-execution warning.
"""

from __future__ import annotations

from typing import Any

__all__ = ["app", "create_app"]


def create_app():
    """Lazy proxy to avoid importing ``.main`` at package import time."""
    from .main import create_app as _create_app

    return _create_app()


def __getattr__(name: str) -> Any:
    if name == "app":
        from .main import app as _app

        return _app
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
