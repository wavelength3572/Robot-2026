"""Parsed-log container and analyzer plugin registry.

A `LogContext` wraps a parsed wpilog and gives analyzers lookup-by-name access
to typed `TimeSeries`. Analyzers register via `@register_analyzer` and return an
`AnalyzerResult` with events, summary numbers, and optional sampled series.
"""

from __future__ import annotations

import bisect
import struct
from bisect import bisect_right
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Iterable

from wpiutil.log import DataLogReader, DataLogRecord


@dataclass
class TimeSeries:
    name: str
    dtype: str
    timestamps_us: list[int] = field(default_factory=list)
    values: list[Any] = field(default_factory=list)

    def __len__(self) -> int:
        return len(self.timestamps_us)

    def __bool__(self) -> bool:
        return bool(self.timestamps_us)

    def value_at(self, ts_us: int, default: Any = None) -> Any:
        """Return the most recent value at or before `ts_us`, else `default`."""
        if not self.timestamps_us:
            return default
        idx = bisect_right(self.timestamps_us, ts_us) - 1
        if idx < 0:
            return default
        return self.values[idx]

    def iter_between(self, t0_us: int, t1_us: int) -> Iterable[tuple[int, Any]]:
        lo = bisect.bisect_left(self.timestamps_us, t0_us)
        hi = bisect.bisect_right(self.timestamps_us, t1_us)
        for i in range(lo, hi):
            yield self.timestamps_us[i], self.values[i]

    def sampled(self, step_us: int) -> tuple[list[float], list[Any]]:
        """Return (seconds_relative_to_t0 of this series, values) downsampled.

        This is used when writing JSON for the dashboard: a 340-second match at
        50 Hz is 17k points per signal which is plenty for Chart.js.
        """
        if not self.timestamps_us:
            return [], []
        out_ts: list[float] = []
        out_val: list[Any] = []
        next_allowed = self.timestamps_us[0]
        for ts, v in zip(self.timestamps_us, self.values):
            if ts >= next_allowed:
                out_ts.append(ts)
                out_val.append(v)
                next_allowed = ts + step_us
        return out_ts, out_val


@dataclass
class AnalyzerResult:
    id: str
    title: str
    events: list[dict] = field(default_factory=list)
    summary: dict[str, Any] = field(default_factory=dict)
    series: dict[str, dict] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)


_REGISTRY: list[tuple[str, str, Callable[["LogContext"], AnalyzerResult]]] = []


def register_analyzer(
    id: str, title: str
) -> Callable[[Callable[["LogContext"], AnalyzerResult]], Callable[["LogContext"], AnalyzerResult]]:
    def _wrap(fn: Callable[["LogContext"], AnalyzerResult]):
        _REGISTRY.append((id, title, fn))
        return fn

    return _wrap


def registered_analyzers() -> list[tuple[str, str, Callable[["LogContext"], AnalyzerResult]]]:
    return list(_REGISTRY)


class LogContext:
    """Lazy wpilog wrapper. Reads the whole log once; analyzers query by name."""

    def __init__(self, path: Path):
        self.path = Path(path)
        self._series: dict[str, TimeSeries] = {}
        self.t0_us: int = 0
        self.t1_us: int = 0
        self.record_count: int = 0
        self._parse()

    def _parse(self) -> None:
        reader = DataLogReader(str(self.path))
        if not reader:
            raise RuntimeError(f"failed to open wpilog: {self.path}")

        entries: dict[int, tuple[str, str]] = {}
        series: dict[str, TimeSeries] = {}
        first_ts = None
        last_ts = 0
        count = 0

        for rec in reader:
            count += 1
            ts = rec.getTimestamp()
            if first_ts is None:
                first_ts = ts
            last_ts = ts

            if rec.isStart():
                s = rec.getStartData()
                entries[s.entry] = (s.name, s.type)
                series.setdefault(s.name, TimeSeries(name=s.name, dtype=s.type))
                continue
            if rec.isFinish() or rec.isSetMetadata() or rec.isControl():
                continue

            meta = entries.get(rec.getEntry())
            if meta is None:
                continue
            name, dtype = meta
            val = _decode(rec, dtype)
            if val is _SENTINEL:
                continue
            ts_list = series[name].timestamps_us
            ts_list.append(ts)
            series[name].values.append(val)

        self._series = series
        self.t0_us = first_ts or 0
        self.t1_us = last_ts or 0
        self.record_count = count

    def signal(self, name: str) -> TimeSeries:
        return self._series.get(name, TimeSeries(name=name, dtype=""))

    def has(self, name: str) -> bool:
        return name in self._series

    def duration_s(self) -> float:
        return (self.t1_us - self.t0_us) / 1e6

    def rel_s(self, ts_us: int) -> float:
        return (ts_us - self.t0_us) / 1e6

    def signal_names(self) -> list[str]:
        return list(self._series.keys())


_SENTINEL = object()


def _decode(rec: DataLogRecord, dtype: str) -> Any:
    """Best-effort decode of primitives we care about. Unknown types → sentinel."""
    try:
        if dtype == "boolean":
            return rec.getBoolean()
        if dtype in ("int64", "int"):
            return rec.getInteger()
        if dtype in ("double", "float"):
            return rec.getDouble()
        if dtype == "string":
            return rec.getString()
        if dtype in ("boolean[]",):
            return list(rec.getBooleanArray())
        if dtype in ("int64[]", "int[]"):
            return list(rec.getIntegerArray())
        if dtype in ("double[]", "float[]"):
            return list(rec.getDoubleArray())
        if dtype in ("string[]",):
            return list(rec.getStringArray())
    except Exception:
        return _SENTINEL
    return _SENTINEL
