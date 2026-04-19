"""Analyzer plugins. Importing this package registers every module under it.

To add a new analyzer, drop a `.py` file next to this one that declares one or
more functions decorated with `@register_analyzer(id=..., title=...)`. It will
be picked up automatically the next time `analyze_wpilog.py` runs.
"""

from __future__ import annotations

import importlib
import pkgutil
from pathlib import Path


def _load_all() -> None:
    pkg_dir = Path(__file__).parent
    for info in pkgutil.iter_modules([str(pkg_dir)]):
        if info.name.startswith("_"):
            continue
        importlib.import_module(f"{__name__}.{info.name}")


_load_all()
