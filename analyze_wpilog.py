#!/usr/bin/env python3
"""
WPILog Analysis Script for FRC Robot 2026
Parses AdvantageKit .wpilog files and generates a comprehensive Markdown report.

Usage: python analyze_wpilog.py "C:\path\to\log\directory" [-o output.md]
"""

import argparse
import math
import os
import struct
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

from wpiutil.log import DataLogReader


# ── Thresholds & Constants ──────────────────────────────────────────────────

CYCLE_OVERRUN_MS = 20.0       # Normal cycle time threshold
CYCLE_SPIKE_MS = 40.0         # Major spike threshold
LOW_VOLTAGE_WARN = 11.0       # Battery voltage warning
LOW_VOLTAGE_CRIT = 10.5       # Battery voltage critical
POSE3D_SIZE = 56              # 7 doubles: x,y,z,qw,qx,qy,qz
POSE_JUMP_THRESHOLD_M = 1.0   # Vision pose jump detection
TURRET_BIG_MOVE_DEG = 90.0    # Large turret angle change threshold
TURRET_BIG_MOVE_WINDOW_US = 1_000_000  # 1 second in microseconds
LAUNCHER_RECOVERY_DROP_PCT = 0.10  # 10% velocity drop = recovery event
INTAKE_CURRENT_SPIKE_A = 5.0  # Intake roller current spike = fuel pickup


# ── Data Structures ─────────────────────────────────────────────────────────

@dataclass
class RunningStats:
    count: int = 0
    min_val: float = float('inf')
    max_val: float = float('-inf')
    sum_val: float = 0.0
    sum_sq: float = 0.0
    min_ts: int = 0
    max_ts: int = 0

    def update(self, val: float, ts: int = 0):
        self.count += 1
        self.sum_val += val
        self.sum_sq += val * val
        if val < self.min_val:
            self.min_val = val
            self.min_ts = ts
        if val > self.max_val:
            self.max_val = val
            self.max_ts = ts

    @property
    def mean(self) -> float:
        return self.sum_val / self.count if self.count > 0 else 0.0

    @property
    def std(self) -> float:
        if self.count < 2:
            return 0.0
        variance = (self.sum_sq / self.count) - (self.mean ** 2)
        return math.sqrt(max(0, variance))

    def summary(self, decimals=2) -> str:
        if self.count == 0:
            return "no data"
        return f"mean={self.mean:.{decimals}f}, min={self.min_val:.{decimals}f}, max={self.max_val:.{decimals}f}, std={self.std:.{decimals}f}, n={self.count}"


@dataclass
class TimeSeries:
    timestamps: list = field(default_factory=list)  # microseconds
    values: list = field(default_factory=list)

    def append(self, ts: int, val):
        self.timestamps.append(ts)
        self.values.append(val)

    def __len__(self):
        return len(self.values)

    def percentile(self, p: float) -> float:
        if not self.values:
            return 0.0
        sorted_vals = sorted(self.values)
        idx = int(len(sorted_vals) * p / 100.0)
        idx = min(idx, len(sorted_vals) - 1)
        return sorted_vals[idx]

    def find_spikes(self, threshold: float) -> list:
        """Return (timestamp, value) for values exceeding threshold."""
        return [(ts, v) for ts, v in zip(self.timestamps, self.values) if v > threshold]


@dataclass
class PhaseTransition:
    timestamp_us: int
    phase: str  # "disabled", "auto", "teleop"


@dataclass
class ConnectionEvent:
    timestamp_us: int
    connected: bool


@dataclass
class ShotEvent:
    timestamp_us: int
    distance_m: float = 0.0
    hood_angle_deg: float = 0.0
    ideal_rpm: float = 0.0
    current_rpm: float = 0.0
    exit_velocity_mps: float = 0.0
    launch_angle_deg: float = 0.0
    azimuth_deg: float = 0.0
    turret_absolute_deg: float = 0.0
    velocity_error: float = 0.0
    velocity_mismatch: float = 0.0
    hood_at_target: bool = False
    launcher_at_setpoint: bool = False
    spindexer_rpm: float = 0.0
    fuel_remaining: int = 0
    file_name: str = ""
    phase: str = ""


@dataclass
class RecoveryEvent:
    timestamp_us: int
    drop_rpm: float  # How far below target
    recovery_time_us: int  # Time to get back to within 5% of target
    spindexer_rpm: float  # Spindexer speed during recovery
    target_rpm: float


@dataclass
class TurretMoveEvent:
    start_ts: int
    end_ts: int
    angle_change_deg: float
    shot_during: bool = False  # Was a shot fired during this move?


@dataclass
class FuelEvent:
    timestamp_us: int
    event_type: str  # "intake", "shot"
    current_amps: float = 0.0


@dataclass
class LogReport:
    filename: str = ""
    file_size_mb: float = 0.0
    record_count: int = 0
    field_count: int = 0
    first_ts: int = 0
    last_ts: int = 0

    # Phase tracking
    phases: list = field(default_factory=list)  # PhaseTransition list
    _last_enabled: bool = False
    _last_auto: bool = False

    # Loop timing
    full_cycle_ms: TimeSeries = field(default_factory=TimeSeries)
    user_code_ms: RunningStats = field(default_factory=RunningStats)
    logger_breakdown: dict = field(default_factory=dict)  # name -> RunningStats

    # Battery / Power
    battery_voltage: TimeSeries = field(default_factory=TimeSeries)
    battery_current: RunningStats = field(default_factory=RunningStats)
    pdp_total_current: RunningStats = field(default_factory=RunningStats)
    brownout_events: list = field(default_factory=list)  # timestamps

    # CAN Bus
    can_utilization: RunningStats = field(default_factory=RunningStats)
    can_off_first: Optional[int] = None
    can_off_last: int = 0
    can_rx_err_first: Optional[int] = None
    can_rx_err_last: int = 0
    can_tx_err_first: Optional[int] = None
    can_tx_err_last: int = 0
    can_tx_full_first: Optional[int] = None
    can_tx_full_last: int = 0

    # Drive modules (0-3)
    drive_current: dict = field(default_factory=lambda: {i: RunningStats() for i in range(4)})
    turn_current: dict = field(default_factory=lambda: {i: RunningStats() for i in range(4)})
    drive_velocity: dict = field(default_factory=lambda: {i: RunningStats() for i in range(4)})
    drive_connected_events: dict = field(default_factory=lambda: {i: [] for i in range(4)})
    turn_connected_events: dict = field(default_factory=lambda: {i: [] for i in range(4)})
    encoder_connected_events: dict = field(default_factory=lambda: {i: [] for i in range(4)})
    gyro_connected_events: list = field(default_factory=list)
    _last_drive_connected: dict = field(default_factory=lambda: {i: None for i in range(4)})
    _last_turn_connected: dict = field(default_factory=lambda: {i: None for i in range(4)})
    _last_encoder_connected: dict = field(default_factory=lambda: {i: None for i in range(4)})
    _last_gyro_connected: Optional[bool] = None

    # Launcher
    launcher_velocity: TimeSeries = field(default_factory=TimeSeries)
    launcher_target_rpm: TimeSeries = field(default_factory=TimeSeries)
    launcher_leader_current: RunningStats = field(default_factory=RunningStats)
    launcher_follower_current: RunningStats = field(default_factory=RunningStats)
    launcher_velocity_error: RunningStats = field(default_factory=RunningStats)
    launcher_velocity_mismatch: RunningStats = field(default_factory=RunningStats)
    launcher_at_setpoint: TimeSeries = field(default_factory=TimeSeries)
    launcher_recovery_boost: TimeSeries = field(default_factory=TimeSeries)
    launcher_leader_temp: RunningStats = field(default_factory=RunningStats)
    launcher_follower_temp: RunningStats = field(default_factory=RunningStats)

    # Hood
    hood_current_angle: TimeSeries = field(default_factory=TimeSeries)
    hood_target_angle: TimeSeries = field(default_factory=TimeSeries)
    hood_at_target: TimeSeries = field(default_factory=TimeSeries)
    hood_current_amps: RunningStats = field(default_factory=RunningStats)
    hood_temp: RunningStats = field(default_factory=RunningStats)

    # Intake
    intake_roller_current: TimeSeries = field(default_factory=TimeSeries)
    intake_roller_rpm: RunningStats = field(default_factory=RunningStats)
    intake_deploy_current: RunningStats = field(default_factory=RunningStats)
    intake_deployed: TimeSeries = field(default_factory=TimeSeries)

    # Spindexer
    spindexer_current: RunningStats = field(default_factory=RunningStats)
    spindexer_rpm: TimeSeries = field(default_factory=TimeSeries)
    spindexer_target_rpm: TimeSeries = field(default_factory=TimeSeries)
    spindexer_temp: RunningStats = field(default_factory=RunningStats)

    # Motivator
    motivator_current: RunningStats = field(default_factory=RunningStats)
    motivator_rpm: TimeSeries = field(default_factory=TimeSeries)
    motivator_temp: RunningStats = field(default_factory=RunningStats)

    # Turret
    turret_current: RunningStats = field(default_factory=RunningStats)
    turret_absolute_angle: TimeSeries = field(default_factory=TimeSeries)
    turret_at_target: TimeSeries = field(default_factory=TimeSeries)
    turret_danger_zone: TimeSeries = field(default_factory=TimeSeries)
    turret_near_limit: TimeSeries = field(default_factory=TimeSeries)

    # Shot tracking
    shot_total: TimeSeries = field(default_factory=TimeSeries)  # cumulative shot count
    shot_events: list = field(default_factory=list)  # ShotEvent list
    fuel_remaining: TimeSeries = field(default_factory=TimeSeries)

    # Current shot state (updated every cycle, captured when shot count increments)
    _shot_state: dict = field(default_factory=lambda: {
        'distance': 0.0, 'hood_angle': 0.0, 'ideal_rpm': 0.0,
        'current_rpm': 0.0, 'exit_vel': 0.0, 'launch_angle': 0.0,
        'azimuth': 0.0, 'turret_abs': 0.0, 'vel_error': 0.0,
        'vel_mismatch': 0.0, 'hood_at_target': False, 'launcher_at_sp': False,
        'spindexer_rpm': 0.0, 'fuel_remaining': 0
    })
    _last_shot_total: int = 0

    # SmartLaunch
    smart_launch_phase: TimeSeries = field(default_factory=TimeSeries)
    smart_launch_ready_all: TimeSeries = field(default_factory=TimeSeries)

    # Recovery tracking
    recovery_events: list = field(default_factory=list)  # RecoveryEvent list

    # Turret moves
    turret_move_events: list = field(default_factory=list)  # TurretMoveEvent list

    # Fuel flow
    fuel_events: list = field(default_factory=list)  # FuelEvent list
    _intake_was_spiking: bool = False

    # Vision per camera (0-3)
    vision_obs_count: dict = field(default_factory=lambda: {i: RunningStats() for i in range(4)})
    vision_accepted_count: dict = field(default_factory=lambda: {i: TimeSeries() for i in range(4)})
    vision_rejected_count: dict = field(default_factory=lambda: {i: TimeSeries() for i in range(4)})
    vision_tag_ids: dict = field(default_factory=lambda: {i: {} for i in range(4)})
    vision_connected: dict = field(default_factory=lambda: {i: [] for i in range(4)})
    _last_vision_connected: dict = field(default_factory=lambda: {i: None for i in range(4)})
    vision_summary_accepted: TimeSeries = field(default_factory=TimeSeries)
    vision_summary_rejected: TimeSeries = field(default_factory=TimeSeries)
    # For pose jump detection: store last accepted pose (x,y) from summary
    _last_accepted_pose_xy: Optional[tuple] = None
    vision_pose_jumps: list = field(default_factory=list)  # (ts, distance_m)

    # Alerts
    alert_errors: set = field(default_factory=set)
    alert_warnings: set = field(default_factory=set)
    alert_infos: set = field(default_factory=set)
    pathplanner_errors: set = field(default_factory=set)
    pathplanner_warnings: set = field(default_factory=set)
    photon_errors: set = field(default_factory=set)
    photon_warnings: set = field(default_factory=set)
    console_messages: list = field(default_factory=list)

    # System
    cpu_temp: RunningStats = field(default_factory=RunningStats)

    # Full field dump: every numeric field -> RunningStats
    all_fields: dict = field(default_factory=dict)  # field_name -> RunningStats

    # Logger queued cycles
    logger_queued_cycles: RunningStats = field(default_factory=RunningStats)

    @property
    def duration_seconds(self) -> float:
        if self.first_ts == 0 or self.last_ts == 0:
            return 0.0
        return (self.last_ts - self.first_ts) / 1_000_000.0

    def phase_at(self, ts: int) -> str:
        """Get the robot phase at a given timestamp."""
        phase = "disabled"
        for p in self.phases:
            if p.timestamp_us <= ts:
                phase = p.phase
            else:
                break
        return phase

    def phase_durations(self) -> dict:
        """Calculate total time spent in each phase."""
        durations = {"disabled": 0, "auto": 0, "teleop": 0}
        if not self.phases:
            return durations
        for i, p in enumerate(self.phases):
            end_ts = self.phases[i + 1].timestamp_us if i + 1 < len(self.phases) else self.last_ts
            dur = end_ts - p.timestamp_us
            if p.phase in durations:
                durations[p.phase] += dur
        return {k: v / 1_000_000.0 for k, v in durations.items()}


# ── Parsing ─────────────────────────────────────────────────────────────────

def ts_sec(ts: int, t0: int) -> float:
    """Convert microsecond timestamp to seconds relative to t0."""
    return (ts - t0) / 1_000_000.0


def parse_log(filepath: str) -> LogReport:
    """Parse a single .wpilog file and return a LogReport."""
    reader = DataLogReader(filepath)
    if not reader.isValid():
        print(f"  WARNING: {filepath} is not a valid wpilog file, skipping")
        report = LogReport()
        report.filename = os.path.basename(filepath)
        return report

    report = LogReport()
    report.filename = os.path.basename(filepath)
    report.file_size_mb = os.path.getsize(filepath) / (1024 * 1024)

    entry_map = {}    # entry_id -> (name, type)
    handlers = {}     # entry_id -> handler callable

    record_count = 0

    def register_handler(entry_id, name, dtype):
        """Register a handler for a field based on its name and type."""
        h = _get_handler(name, dtype, report)
        if h:
            handlers[entry_id] = h

    for record in reader:
        if record.isStart():
            d = record.getStartData()
            entry_map[d.entry] = (d.name, d.type)
            report.field_count += 1
            register_handler(d.entry, d.name, d.type)
            continue

        if record.isControl():
            continue

        record_count += 1
        ts = record.getTimestamp()

        if report.first_ts == 0:
            report.first_ts = ts
        report.last_ts = ts

        entry_id = record.getEntry()
        handler = handlers.get(entry_id)
        if handler:
            try:
                handler(record, ts)
            except Exception:
                pass  # Silently skip malformed records

        # Full field dump: track all numeric fields
        entry_info = entry_map.get(entry_id)
        if entry_info:
            name, dtype = entry_info
            if dtype == 'double':
                try:
                    val = record.getDouble()
                    if name not in report.all_fields:
                        report.all_fields[name] = RunningStats()
                    report.all_fields[name].update(val, ts)
                except Exception:
                    pass
            elif dtype == 'float':
                try:
                    val = record.getFloat()
                    if name not in report.all_fields:
                        report.all_fields[name] = RunningStats()
                    report.all_fields[name].update(val, ts)
                except Exception:
                    pass
            elif dtype == 'int64':
                try:
                    val = record.getInteger()
                    if name not in report.all_fields:
                        report.all_fields[name] = RunningStats()
                    report.all_fields[name].update(float(val), ts)
                except Exception:
                    pass

    report.record_count = record_count

    # Post-processing: detect recovery events from launcher time series
    _detect_recovery_events(report)
    # Post-processing: detect turret big moves
    _detect_turret_moves(report)
    # Post-processing: detect fuel intake events
    _detect_fuel_events(report)

    return report


def _get_handler(name: str, dtype: str, report: LogReport):
    """Return a handler closure for the given field, or None if we don't care about it."""

    # ── Phase tracking ──
    if name == '/DriverStation/Enabled':
        def h(record, ts):
            val = record.getBoolean()
            if val != report._last_enabled:
                report._last_enabled = val
                phase = "disabled"
                if val:
                    phase = "auto" if report._last_auto else "teleop"
                report.phases.append(PhaseTransition(ts, phase))
        return h
    if name == '/DriverStation/Autonomous':
        def h(record, ts):
            val = record.getBoolean()
            if val != report._last_auto:
                report._last_auto = val
                if report._last_enabled:
                    phase = "auto" if val else "teleop"
                    report.phases.append(PhaseTransition(ts, phase))
        return h

    # ── Loop timing ──
    if name == '/RealOutputs/LoggedRobot/FullCycleMS':
        def h(record, ts):
            val = record.getDouble()
            report.full_cycle_ms.append(ts, val)
        return h
    if name == '/RealOutputs/LoggedRobot/UserCodeMS':
        def h(record, ts):
            report.user_code_ms.update(record.getDouble(), ts)
        return h
    if name.startswith('/RealOutputs/Logger/') and name.endswith('MS'):
        key = name.split('/')[-1]
        def h(record, ts, k=key):
            if k not in report.logger_breakdown:
                report.logger_breakdown[k] = RunningStats()
            report.logger_breakdown[k].update(record.getDouble(), ts)
        return h
    if name == '/RealOutputs/Logger/QueuedCycles':
        def h(record, ts):
            report.logger_queued_cycles.update(float(record.getInteger()), ts)
        return h

    # ── Battery / Power ──
    if name == '/SystemStats/BatteryVoltage':
        def h(record, ts):
            val = record.getDouble()
            report.battery_voltage.append(ts, val)
        return h
    if name == '/SystemStats/BatteryCurrent':
        def h(record, ts):
            report.battery_current.update(record.getDouble(), ts)
        return h
    if name == '/PowerDistribution/TotalCurrent':
        def h(record, ts):
            report.pdp_total_current.update(record.getDouble(), ts)
        return h
    if name == '/SystemStats/BrownedOut':
        def h(record, ts):
            if record.getBoolean():
                report.brownout_events.append(ts)
        return h

    # ── CAN Bus ──
    if name == '/SystemStats/CANBus/Utilization' and dtype == 'float':
        def h(record, ts):
            report.can_utilization.update(record.getFloat(), ts)
        return h
    if name == '/SystemStats/CANBus/OffCount':
        def h(record, ts):
            val = record.getInteger()
            if report.can_off_first is None:
                report.can_off_first = val
            report.can_off_last = val
        return h
    if name == '/SystemStats/CANBus/ReceiveErrorCount':
        def h(record, ts):
            val = record.getInteger()
            if report.can_rx_err_first is None:
                report.can_rx_err_first = val
            report.can_rx_err_last = val
        return h
    if name == '/SystemStats/CANBus/TransmitErrorCount':
        def h(record, ts):
            val = record.getInteger()
            if report.can_tx_err_first is None:
                report.can_tx_err_first = val
            report.can_tx_err_last = val
        return h
    if name == '/SystemStats/CANBus/TxFullCount':
        def h(record, ts):
            val = record.getInteger()
            if report.can_tx_full_first is None:
                report.can_tx_full_first = val
            report.can_tx_full_last = val
        return h

    # ── Drive modules ──
    for mod in range(4):
        prefix = f'/Drive/Module{mod}/'
        if name == prefix + 'DriveCurrentAmps':
            def h(record, ts, m=mod):
                report.drive_current[m].update(record.getDouble(), ts)
            return h
        if name == prefix + 'TurnCurrentAmps':
            def h(record, ts, m=mod):
                report.turn_current[m].update(record.getDouble(), ts)
            return h
        if name == prefix + 'DriveVelocityRadPerSec':
            def h(record, ts, m=mod):
                report.drive_velocity[m].update(record.getDouble(), ts)
            return h
        if name == prefix + 'DriveConnected':
            def h(record, ts, m=mod):
                val = record.getBoolean()
                if val != report._last_drive_connected[m]:
                    report._last_drive_connected[m] = val
                    report.drive_connected_events[m].append(ConnectionEvent(ts, val))
            return h
        if name == prefix + 'TurnConnected':
            def h(record, ts, m=mod):
                val = record.getBoolean()
                if val != report._last_turn_connected[m]:
                    report._last_turn_connected[m] = val
                    report.turn_connected_events[m].append(ConnectionEvent(ts, val))
            return h
        if name == prefix + 'TurnEncoderConnected':
            def h(record, ts, m=mod):
                val = record.getBoolean()
                if val != report._last_encoder_connected[m]:
                    report._last_encoder_connected[m] = val
                    report.encoder_connected_events[m].append(ConnectionEvent(ts, val))
            return h

    if name == '/Drive/Gyro/Connected':
        def h(record, ts):
            val = record.getBoolean()
            if val != report._last_gyro_connected:
                report._last_gyro_connected = val
                report.gyro_connected_events.append(ConnectionEvent(ts, val))
        return h

    # ── Launcher ──
    if name == '/RealOutputs/Launcher/currentVelocity':
        def h(record, ts):
            val = record.getDouble()
            report.launcher_velocity.append(ts, val)
            report._shot_state['current_rpm'] = val
        return h
    if name == '/Launcher/TargetVelocityRPM':
        def h(record, ts):
            val = record.getDouble()
            report.launcher_target_rpm.append(ts, val)
        return h
    if name == '/Launcher/LeaderCurrentAmps':
        def h(record, ts):
            report.launcher_leader_current.update(record.getDouble(), ts)
        return h
    if name == '/Launcher/FollowerCurrentAmps':
        def h(record, ts):
            report.launcher_follower_current.update(record.getDouble(), ts)
        return h
    if name == '/RealOutputs/Launcher/velocityError':
        def h(record, ts):
            val = record.getDouble()
            report.launcher_velocity_error.update(val, ts)
            report._shot_state['vel_error'] = val
        return h
    if name == '/RealOutputs/Launcher/velocityMismatch':
        def h(record, ts):
            val = record.getDouble()
            report.launcher_velocity_mismatch.update(val, ts)
            report._shot_state['vel_mismatch'] = val
        return h
    if name == '/Launcher/AtSetpoint':
        def h(record, ts):
            val = record.getBoolean()
            report.launcher_at_setpoint.append(ts, 1.0 if val else 0.0)
            report._shot_state['launcher_at_sp'] = val
        return h
    if name == '/RealOutputs/Launcher/RecoveryBoostActive':
        def h(record, ts):
            report.launcher_recovery_boost.append(ts, 1.0 if record.getBoolean() else 0.0)
        return h
    if name == '/Launcher/LeaderTempCelsius':
        def h(record, ts):
            report.launcher_leader_temp.update(record.getDouble(), ts)
        return h
    if name == '/Launcher/FollowerTempCelsius':
        def h(record, ts):
            report.launcher_follower_temp.update(record.getDouble(), ts)
        return h

    # ── Hood ──
    if name == '/Hood/CurrentAngleDeg':
        def h(record, ts):
            report.hood_current_angle.append(ts, record.getDouble())
        return h
    if name == '/Hood/TargetAngleDeg':
        def h(record, ts):
            report.hood_target_angle.append(ts, record.getDouble())
        return h
    if name == '/Hood/AtTarget':
        def h(record, ts):
            val = record.getBoolean()
            report.hood_at_target.append(ts, 1.0 if val else 0.0)
            report._shot_state['hood_at_target'] = val
        return h
    if name == '/Hood/CurrentAmps':
        def h(record, ts):
            report.hood_current_amps.update(record.getDouble(), ts)
        return h
    if name == '/Hood/TempCelsius':
        def h(record, ts):
            report.hood_temp.update(record.getDouble(), ts)
        return h

    # ── Intake ──
    if name == '/Intake/RollerCurrentAmps':
        def h(record, ts):
            val = record.getDouble()
            report.intake_roller_current.append(ts, val)
        return h
    if name == '/Intake/RollerVelocityRPM':
        def h(record, ts):
            report.intake_roller_rpm.update(record.getDouble(), ts)
        return h
    if name == '/Intake/DeployCurrentAmps':
        def h(record, ts):
            report.intake_deploy_current.update(record.getDouble(), ts)
        return h
    if name == '/RealOutputs/Intake/IsDeployed':
        def h(record, ts):
            report.intake_deployed.append(ts, 1.0 if record.getBoolean() else 0.0)
        return h

    # ── Spindexer ──
    if name == '/Spindexer/CurrentAmps':
        def h(record, ts):
            report.spindexer_current.update(record.getDouble(), ts)
        return h
    if name == '/Spindexer/WheelRPM':
        def h(record, ts):
            val = record.getDouble()
            report.spindexer_rpm.append(ts, val)
            report._shot_state['spindexer_rpm'] = val
        return h
    if name == '/Spindexer/TargetRPM':
        def h(record, ts):
            report.spindexer_target_rpm.append(ts, record.getDouble())
        return h
    if name == '/Spindexer/TempCelsius':
        def h(record, ts):
            report.spindexer_temp.update(record.getDouble(), ts)
        return h

    # ── Motivator ──
    if name == '/Motivator/CurrentAmps':
        def h(record, ts):
            report.motivator_current.update(record.getDouble(), ts)
        return h
    if name == '/Motivator/WheelRPM':
        def h(record, ts):
            report.motivator_rpm.append(ts, record.getDouble())
        return h
    if name == '/Motivator/TempCelsius':
        def h(record, ts):
            report.motivator_temp.update(record.getDouble(), ts)
        return h

    # ── Turret ──
    if name == '/Turret/CurrentAmps':
        def h(record, ts):
            report.turret_current.update(record.getDouble(), ts)
        return h
    if name == '/RealOutputs/Turret/AbsoluteAngle':
        def h(record, ts):
            val = record.getDouble()
            report.turret_absolute_angle.append(ts, val)
            report._shot_state['turret_abs'] = val
        return h
    if name == '/RealOutputs/Turret/AtTarget':
        def h(record, ts):
            report.turret_at_target.append(ts, 1.0 if record.getBoolean() else 0.0)
        return h
    if name == '/RealOutputs/Turret/Safety/InDangerZone':
        def h(record, ts):
            report.turret_danger_zone.append(ts, 1.0 if record.getBoolean() else 0.0)
        return h
    if name == '/RealOutputs/Turret/Safety/NearLimit':
        def h(record, ts):
            report.turret_near_limit.append(ts, 1.0 if record.getBoolean() else 0.0)
        return h

    # ── Shot tracking ──
    if name == '/RealOutputs/Match/ShotLog/TotalShots':
        def h(record, ts):
            val = record.getInteger()
            report.shot_total.append(ts, val)
            if val > report._last_shot_total and report._last_shot_total >= 0:
                # Shot happened! Capture current state
                for _ in range(val - report._last_shot_total):
                    evt = ShotEvent(
                        timestamp_us=ts,
                        distance_m=report._shot_state['distance'],
                        hood_angle_deg=report._shot_state['hood_angle'],
                        ideal_rpm=report._shot_state['ideal_rpm'],
                        current_rpm=report._shot_state['current_rpm'],
                        exit_velocity_mps=report._shot_state['exit_vel'],
                        launch_angle_deg=report._shot_state['launch_angle'],
                        azimuth_deg=report._shot_state['azimuth'],
                        turret_absolute_deg=report._shot_state['turret_abs'],
                        velocity_error=report._shot_state['vel_error'],
                        velocity_mismatch=report._shot_state['vel_mismatch'],
                        hood_at_target=report._shot_state['hood_at_target'],
                        launcher_at_setpoint=report._shot_state['launcher_at_sp'],
                        spindexer_rpm=report._shot_state['spindexer_rpm'],
                        fuel_remaining=report._shot_state['fuel_remaining'],
                        file_name=report.filename,
                        phase=report.phase_at(ts)
                    )
                    report.shot_events.append(evt)
            report._last_shot_total = val
        return h
    if name == '/RealOutputs/Match/ShotLog/FuelRemaining':
        def h(record, ts):
            val = record.getInteger()
            report.fuel_remaining.append(ts, val)
            report._shot_state['fuel_remaining'] = val
        return h

    # Shot state fields (updated continuously, captured when shot fires)
    if name == '/RealOutputs/Turret/Shot/DistanceToTargetM':
        def h(record, ts):
            report._shot_state['distance'] = record.getDouble()
        return h
    if name == '/RealOutputs/Turret/Shot/HoodAngleDeg':
        def h(record, ts):
            report._shot_state['hood_angle'] = record.getDouble()
        return h
    if name == '/RealOutputs/Turret/Shot/IdealRPM':
        def h(record, ts):
            report._shot_state['ideal_rpm'] = record.getDouble()
        return h
    if name == '/RealOutputs/Turret/Shot/ExitVelocityMps':
        def h(record, ts):
            report._shot_state['exit_vel'] = record.getDouble()
        return h
    if name == '/RealOutputs/Turret/Shot/LaunchAngleDeg':
        def h(record, ts):
            report._shot_state['launch_angle'] = record.getDouble()
        return h
    if name == '/RealOutputs/Turret/Shot/AzimuthDeg':
        def h(record, ts):
            report._shot_state['azimuth'] = record.getDouble()
        return h

    # ── SmartLaunch ──
    if name == '/RealOutputs/SmartLaunch/Phase' and dtype == 'string':
        def h(record, ts):
            report.smart_launch_phase.append(ts, record.getString())
        return h
    if name == '/RealOutputs/SmartLaunch/Ready/All':
        def h(record, ts):
            report.smart_launch_ready_all.append(ts, 1.0 if record.getBoolean() else 0.0)
        return h

    # ── Vision per camera ──
    for cam in range(4):
        if name == f'/RealOutputs/Vision/Camera{cam}/ObservationCount':
            def h(record, ts, c=cam):
                report.vision_obs_count[c].update(float(record.getInteger()), ts)
            return h
        if name == f'/Vision/Camera{cam}/TagIds' and dtype == 'int64[]':
            def h(record, ts, c=cam):
                try:
                    ids = record.getIntegerArray()
                    for tid in ids:
                        report.vision_tag_ids[c][tid] = report.vision_tag_ids[c].get(tid, 0) + 1
                except Exception:
                    pass
            return h
        if name == f'/Vision/Camera{cam}/Connected':
            def h(record, ts, c=cam):
                val = record.getBoolean()
                if val != report._last_vision_connected[c]:
                    report._last_vision_connected[c] = val
                    report.vision_connected[c].append(ConnectionEvent(ts, val))
            return h
        if name == f'/RealOutputs/Vision/Camera{cam}/RobotPosesAccepted' and 'struct:Pose3d[]' in dtype:
            def h(record, ts, c=cam):
                raw = record.getRaw()
                count = len(raw) // POSE3D_SIZE if raw else 0
                report.vision_accepted_count[c].append(ts, count)
            return h
        if name == f'/RealOutputs/Vision/Camera{cam}/RobotPosesRejected' and 'struct:Pose3d[]' in dtype:
            def h(record, ts, c=cam):
                raw = record.getRaw()
                count = len(raw) // POSE3D_SIZE if raw else 0
                report.vision_rejected_count[c].append(ts, count)
            return h

    # Vision summary
    if name == '/RealOutputs/Vision/Summary/RobotPosesAccepted' and 'struct:Pose3d[]' in dtype:
        def h(record, ts):
            raw = record.getRaw()
            count = len(raw) // POSE3D_SIZE if raw else 0
            report.vision_summary_accepted.append(ts, count)
            # Pose jump detection
            if count > 0 and len(raw) >= 24:
                x, y, z = struct.unpack_from('<3d', raw, 0)
                if report._last_accepted_pose_xy is not None:
                    dx = x - report._last_accepted_pose_xy[0]
                    dy = y - report._last_accepted_pose_xy[1]
                    dist = math.sqrt(dx * dx + dy * dy)
                    if dist > POSE_JUMP_THRESHOLD_M:
                        report.vision_pose_jumps.append((ts, dist))
                report._last_accepted_pose_xy = (x, y)
        return h
    if name == '/RealOutputs/Vision/Summary/RobotPosesRejected' and 'struct:Pose3d[]' in dtype:
        def h(record, ts):
            raw = record.getRaw()
            count = len(raw) // POSE3D_SIZE if raw else 0
            report.vision_summary_rejected.append(ts, count)
        return h

    # ── Alerts ──
    if name == '/RealOutputs/Alerts/errors' and dtype == 'string[]':
        def h(record, ts):
            for s in record.getStringArray():
                if s:
                    report.alert_errors.add(s)
        return h
    if name == '/RealOutputs/Alerts/warnings' and dtype == 'string[]':
        def h(record, ts):
            for s in record.getStringArray():
                if s:
                    report.alert_warnings.add(s)
        return h
    if name == '/RealOutputs/Alerts/infos' and dtype == 'string[]':
        def h(record, ts):
            for s in record.getStringArray():
                if s:
                    report.alert_infos.add(s)
        return h
    if name == '/RealOutputs/PathPlanner/errors' and dtype == 'string[]':
        def h(record, ts):
            for s in record.getStringArray():
                if s:
                    report.pathplanner_errors.add(s)
        return h
    if name == '/RealOutputs/PathPlanner/warnings' and dtype == 'string[]':
        def h(record, ts):
            for s in record.getStringArray():
                if s:
                    report.pathplanner_warnings.add(s)
        return h
    if name == '/RealOutputs/PhotonAlerts/errors' and dtype == 'string[]':
        def h(record, ts):
            for s in record.getStringArray():
                if s:
                    report.photon_errors.add(s)
        return h
    if name == '/RealOutputs/PhotonAlerts/warnings' and dtype == 'string[]':
        def h(record, ts):
            for s in record.getStringArray():
                if s:
                    report.photon_warnings.add(s)
        return h
    if name == '/RealOutputs/Console' and dtype == 'string':
        def h(record, ts):
            val = record.getString()
            if val and len(report.console_messages) < 500:
                report.console_messages.append(val)
        return h

    # ── System ──
    if name == '/SystemStats/CPUTempCelsius':
        def h(record, ts):
            report.cpu_temp.update(record.getDouble(), ts)
        return h

    return None  # We don't care about this field


# ── Post-processing ─────────────────────────────────────────────────────────

def _detect_recovery_events(report: LogReport):
    """Detect launcher recovery events from velocity time series."""
    vel = report.launcher_velocity
    tgt = report.launcher_target_rpm
    if len(vel) < 10 or len(tgt) < 1:
        return

    # Build a target lookup: for each velocity timestamp, find nearest target
    tgt_vals = list(zip(tgt.timestamps, tgt.values))
    tgt_idx = 0

    in_recovery = False
    recovery_start_ts = 0
    recovery_drop = 0.0
    recovery_target = 0.0

    for i in range(len(vel)):
        ts = vel.timestamps[i]
        v = vel.values[i]

        # Advance target index
        while tgt_idx < len(tgt_vals) - 1 and tgt_vals[tgt_idx + 1][0] <= ts:
            tgt_idx += 1
        target = tgt_vals[tgt_idx][1] if tgt_idx < len(tgt_vals) else 0.0

        if target <= 100:  # Launcher not active
            in_recovery = False
            continue

        drop_pct = (target - v) / target if target > 0 else 0

        if not in_recovery and drop_pct > LAUNCHER_RECOVERY_DROP_PCT:
            in_recovery = True
            recovery_start_ts = ts
            recovery_drop = target - v
            recovery_target = target
        elif in_recovery and drop_pct <= 0.05:  # Recovered to within 5%
            # Find spindexer RPM at recovery start
            spindexer_rpm = 0.0
            if report.spindexer_rpm.timestamps:
                for j in range(len(report.spindexer_rpm.timestamps)):
                    if report.spindexer_rpm.timestamps[j] >= recovery_start_ts:
                        spindexer_rpm = report.spindexer_rpm.values[j]
                        break

            report.recovery_events.append(RecoveryEvent(
                timestamp_us=recovery_start_ts,
                drop_rpm=recovery_drop,
                recovery_time_us=ts - recovery_start_ts,
                spindexer_rpm=spindexer_rpm,
                target_rpm=recovery_target
            ))
            in_recovery = False


def _detect_turret_moves(report: LogReport):
    """Detect large turret angle changes."""
    angle = report.turret_absolute_angle
    if len(angle) < 10:
        return

    # Sliding window: check angle change over 1 second windows
    j = 0
    for i in range(len(angle)):
        while j < len(angle) and (angle.timestamps[j] - angle.timestamps[i]) < TURRET_BIG_MOVE_WINDOW_US:
            j += 1
        if j >= len(angle):
            break
        for k in range(i, min(j, len(angle))):
            change = abs(angle.values[k] - angle.values[i])
            if change > TURRET_BIG_MOVE_DEG:
                # Check if any shot happened during this window
                shot_during = False
                for evt in report.shot_events:
                    if angle.timestamps[i] <= evt.timestamp_us <= angle.timestamps[min(k, len(angle) - 1)]:
                        shot_during = True
                        break
                report.turret_move_events.append(TurretMoveEvent(
                    start_ts=angle.timestamps[i],
                    end_ts=angle.timestamps[k],
                    angle_change_deg=change,
                    shot_during=shot_during
                ))
                break  # Only record one event per starting point


def _detect_fuel_events(report: LogReport):
    """Detect fuel intake events from roller current spikes."""
    roller = report.intake_roller_current
    deployed = report.intake_deployed
    if len(roller) < 10:
        return

    # Build deployed state lookup
    deployed_at = {}
    if deployed.timestamps:
        for i in range(len(deployed)):
            deployed_at[deployed.timestamps[i]] = deployed.values[i] > 0.5

    was_spiking = False
    for i in range(len(roller)):
        ts = roller.timestamps[i]
        val = roller.values[i]

        if val > INTAKE_CURRENT_SPIKE_A and not was_spiking:
            report.fuel_events.append(FuelEvent(ts, "intake", val))
            was_spiking = True
        elif val <= INTAKE_CURRENT_SPIKE_A * 0.7:  # Hysteresis
            was_spiking = False


# ── Report Generation ───────────────────────────────────────────────────────

def generate_report(reports: list, output_path: str):
    """Generate comprehensive Markdown report."""
    lines = []
    w = lines.append

    w("# WPILog Analysis Report - GVSU Practice 2/28")
    w(f"*Generated: {time.strftime('%Y-%m-%d %H:%M:%S')}*")
    w(f"*Files analyzed: {len(reports)}*")
    w("")

    # ── All shots across files (lookup table data) ──
    all_shots = []
    for r in reports:
        all_shots.extend(r.shot_events)

    if all_shots:
        w("---")
        w("# SHOT DATA (Lookup Table Seed)")
        w("")
        w("| # | File | Phase | Time(s) | Dist(m) | Hood(deg) | IdealRPM | ActualRPM | VelErr | ExitVel(m/s) | LaunchAng(deg) | Azimuth(deg) | TurretAbs(deg) | HoodReady | LauncherReady | SpindexerRPM | FuelLeft |")
        w("|---|------|-------|---------|---------|-----------|----------|-----------|--------|--------------|----------------|--------------|----------------|-----------|---------------|--------------|----------|")
        for i, s in enumerate(all_shots, 1):
            t = ts_sec(s.timestamp_us, _find_t0(reports, s.file_name))
            w(f"| {i} | {s.file_name[:25]} | {s.phase} | {t:.1f} | {s.distance_m:.2f} | {s.hood_angle_deg:.1f} | {s.ideal_rpm:.0f} | {s.current_rpm:.0f} | {s.velocity_error:.0f} | {s.exit_velocity_mps:.1f} | {s.launch_angle_deg:.1f} | {s.azimuth_deg:.1f} | {s.turret_absolute_deg:.1f} | {'Y' if s.hood_at_target else 'N'} | {'Y' if s.launcher_at_setpoint else 'N'} | {s.spindexer_rpm:.0f} | {s.fuel_remaining} |")
        w("")
        # Shots not ready summary
        not_ready_hood = sum(1 for s in all_shots if not s.hood_at_target)
        not_ready_launcher = sum(1 for s in all_shots if not s.launcher_at_setpoint)
        w(f"**Total shots: {len(all_shots)}** | Hood not ready: {not_ready_hood} | Launcher not ready: {not_ready_launcher}")
        w("")

    # ── Per-file reports ──
    for r in reports:
        t0 = r.first_ts
        w("---")
        w(f"# Log: {r.filename}")
        w(f"**Size**: {r.file_size_mb:.1f} MB | **Duration**: {r.duration_seconds:.1f}s | **Records**: {r.record_count:,} | **Fields**: {r.field_count}")
        w("")

        # Phases
        pd = r.phase_durations()
        w("## Phases")
        w(f"- Disabled: {pd['disabled']:.1f}s | Auto: {pd['auto']:.1f}s | Teleop: {pd['teleop']:.1f}s")
        if r.phases:
            w("")
            w("| Time(s) | Phase |")
            w("|---------|-------|")
            for p in r.phases:
                w(f"| {ts_sec(p.timestamp_us, t0):.1f} | {p.phase} |")
        w("")

        # Loop Timing
        w("## Loop Timing")
        if len(r.full_cycle_ms) > 0:
            vals = r.full_cycle_ms.values
            mean_ms = sum(vals) / len(vals)
            p95 = r.full_cycle_ms.percentile(95)
            p99 = r.full_cycle_ms.percentile(99)
            max_ms = max(vals)
            overrun_pct = sum(1 for v in vals if v > CYCLE_OVERRUN_MS) / len(vals) * 100
            spike_pct = sum(1 for v in vals if v > CYCLE_SPIKE_MS) / len(vals) * 100

            w(f"| Metric | Value |")
            w(f"|--------|-------|")
            w(f"| Mean cycle | {mean_ms:.1f} ms |")
            w(f"| P95 | {p95:.1f} ms |")
            w(f"| P99 | {p99:.1f} ms |")
            w(f"| Max | {max_ms:.1f} ms |")
            w(f"| Overruns (>{CYCLE_OVERRUN_MS}ms) | {overrun_pct:.1f}% |")
            w(f"| Major spikes (>{CYCLE_SPIKE_MS}ms) | {spike_pct:.1f}% |")
            if r.user_code_ms.count > 0:
                w(f"| UserCode mean | {r.user_code_ms.mean:.1f} ms |")
            w("")

            # Spike list
            spikes = r.full_cycle_ms.find_spikes(CYCLE_SPIKE_MS)
            if spikes:
                w(f"### Spikes >{CYCLE_SPIKE_MS}ms (showing up to 20)")
                w("| Time(s) | CycleMS | Phase |")
                w("|---------|---------|-------|")
                for ts_us, val in spikes[:20]:
                    w(f"| {ts_sec(ts_us, t0):.2f} | {val:.1f} | {r.phase_at(ts_us)} |")
                w("")

            # Logger breakdown
            if r.logger_breakdown:
                w("### Logger Breakdown (mean ms)")
                w("| Component | Mean | Max |")
                w("|-----------|------|-----|")
                for k, stats in sorted(r.logger_breakdown.items(), key=lambda x: -x[1].mean):
                    w(f"| {k} | {stats.mean:.2f} | {stats.max_val:.2f} |")
                if r.logger_queued_cycles.count > 0:
                    w(f"| QueuedCycles | mean={r.logger_queued_cycles.mean:.1f} | max={r.logger_queued_cycles.max_val:.0f} |")
                w("")
        else:
            w("*No loop timing data*")
            w("")

        # Battery & Power
        w("## Battery & Power")
        if len(r.battery_voltage) > 0:
            bv = r.battery_voltage.values
            w(f"| Metric | Value |")
            w(f"|--------|-------|")
            w(f"| Mean voltage | {sum(bv)/len(bv):.2f} V |")
            w(f"| Min voltage | {min(bv):.2f} V (at {ts_sec(r.battery_voltage.timestamps[bv.index(min(bv))], t0):.1f}s) |")
            w(f"| Max voltage | {max(bv):.2f} V |")
            if r.battery_current.count > 0:
                w(f"| Mean current | {r.battery_current.mean:.1f} A |")
                w(f"| Max current | {r.battery_current.max_val:.1f} A |")
            if r.pdp_total_current.count > 0:
                w(f"| PDH total current (mean) | {r.pdp_total_current.mean:.1f} A |")
                w(f"| PDH total current (max) | {r.pdp_total_current.max_val:.1f} A |")
            w(f"| Brownout events | {len(r.brownout_events)} |")

            # Low voltage periods
            low_v = [(ts, v) for ts, v in zip(r.battery_voltage.timestamps, bv) if v < LOW_VOLTAGE_WARN]
            if low_v:
                w(f"| Samples below {LOW_VOLTAGE_WARN}V | {len(low_v)} |")
            w("")
        else:
            w("*No battery data*")
            w("")

        # CAN Bus
        w("## CAN Bus Health")
        if r.can_utilization.count > 0:
            w(f"| Metric | Value |")
            w(f"|--------|-------|")
            w(f"| Mean utilization | {r.can_utilization.mean*100:.1f}% |")
            w(f"| Max utilization | {r.can_utilization.max_val*100:.1f}% |")
            can_off_delta = (r.can_off_last - r.can_off_first) if r.can_off_first is not None else 0
            can_rx_delta = (r.can_rx_err_last - r.can_rx_err_first) if r.can_rx_err_first is not None else 0
            can_tx_delta = (r.can_tx_err_last - r.can_tx_err_first) if r.can_tx_err_first is not None else 0
            can_full_delta = (r.can_tx_full_last - r.can_tx_full_first) if r.can_tx_full_first is not None else 0
            w(f"| Off count (delta) | {can_off_delta} |")
            w(f"| Rx errors (delta) | {can_rx_delta} |")
            w(f"| Tx errors (delta) | {can_tx_delta} |")
            w(f"| Tx full (delta) | {can_full_delta} |")
            w("")
        else:
            w("*No CAN data*")
            w("")

        # Drive Module Health
        w("## Drive Module Health")
        w("### Current Draw (Amps)")
        w("| Module | Drive Mean | Drive Max | Turn Mean | Turn Max | Drive Vel Mean (rad/s) |")
        w("|--------|------------|-----------|-----------|----------|----------------------|")
        for mod in range(4):
            dc = r.drive_current[mod]
            tc = r.turn_current[mod]
            dv = r.drive_velocity[mod]
            w(f"| {mod} | {dc.mean:.1f} | {dc.max_val:.1f} | {tc.mean:.1f} | {tc.max_val:.1f} | {dv.mean:.1f} |")
        w("")

        # Asymmetry check
        drive_means = [r.drive_current[i].mean for i in range(4) if r.drive_current[i].count > 0]
        if drive_means:
            avg = sum(drive_means) / len(drive_means)
            if avg > 0:
                for mod in range(4):
                    if r.drive_current[mod].count > 0:
                        pct_diff = abs(r.drive_current[mod].mean - avg) / avg * 100
                        if pct_diff > 30:
                            w(f"**WARNING: Module {mod} drive current is {pct_diff:.0f}% off average ({r.drive_current[mod].mean:.1f}A vs avg {avg:.1f}A)**")

        # Connection events
        has_disconnects = False
        for mod in range(4):
            for events, name in [(r.drive_connected_events[mod], f"Module{mod}/Drive"),
                                  (r.turn_connected_events[mod], f"Module{mod}/Turn"),
                                  (r.encoder_connected_events[mod], f"Module{mod}/Encoder")]:
                disconnects = [e for e in events if not e.connected]
                if disconnects:
                    if not has_disconnects:
                        w("### Disconnection Events")
                        has_disconnects = True
                    for e in disconnects:
                        w(f"- **{name}** disconnected at {ts_sec(e.timestamp_us, t0):.2f}s")
        if r.gyro_connected_events:
            disconnects = [e for e in r.gyro_connected_events if not e.connected]
            if disconnects:
                if not has_disconnects:
                    w("### Disconnection Events")
                    has_disconnects = True
                for e in disconnects:
                    w(f"- **Gyro** disconnected at {ts_sec(e.timestamp_us, t0):.2f}s")
        w("")

        # Launcher
        w("## Launcher")
        if r.launcher_leader_current.count > 0:
            w("| Metric | Value |")
            w("|--------|-------|")
            w(f"| Leader current | {r.launcher_leader_current.summary(1)} |")
            w(f"| Follower current | {r.launcher_follower_current.summary(1)} |")
            if r.launcher_velocity_error.count > 0:
                w(f"| Velocity error | {r.launcher_velocity_error.summary(1)} |")
            if r.launcher_velocity_mismatch.count > 0:
                w(f"| Leader/follower mismatch | {r.launcher_velocity_mismatch.summary(1)} |")
            if r.launcher_leader_temp.count > 0:
                w(f"| Leader temp | {r.launcher_leader_temp.summary(1)} |")
            if r.launcher_follower_temp.count > 0:
                w(f"| Follower temp | {r.launcher_follower_temp.summary(1)} |")

            # Recovery boost usage
            if len(r.launcher_recovery_boost) > 0:
                boost_on = sum(1 for v in r.launcher_recovery_boost.values if v > 0.5)
                boost_pct = boost_on / len(r.launcher_recovery_boost) * 100
                w(f"| Recovery boost active | {boost_pct:.1f}% of samples |")
            if r.shot_events:
                launcher_ready_shots = sum(1 for s in r.shot_events if s.launcher_at_setpoint)
                w(f"| Launcher ready at shot time | {launcher_ready_shots}/{len(r.shot_events)} shots |")
            w("")

            # Recovery events
            if r.recovery_events:
                w("### Launcher Recovery Events")
                w(f"**{len(r.recovery_events)} recovery events detected**")
                recovery_times_ms = [e.recovery_time_us / 1000 for e in r.recovery_events]
                if recovery_times_ms:
                    w(f"- Mean recovery time: {sum(recovery_times_ms)/len(recovery_times_ms):.0f} ms")
                    w(f"- Max recovery time: {max(recovery_times_ms):.0f} ms")
                    w(f"- Min recovery time: {min(recovery_times_ms):.0f} ms")
                w("")
                w("| Time(s) | Drop(RPM) | Recovery(ms) | SpindexerRPM | TargetRPM |")
                w("|---------|-----------|--------------|--------------|-----------|")
                for e in r.recovery_events[:30]:
                    w(f"| {ts_sec(e.timestamp_us, t0):.1f} | {e.drop_rpm:.0f} | {e.recovery_time_us/1000:.0f} | {e.spindexer_rpm:.0f} | {e.target_rpm:.0f} |")
                w("")
        else:
            w("*No launcher data*")
            w("")

        # Hood
        w("## Hood")
        if len(r.hood_current_angle) > 0:
            w("| Metric | Value |")
            w("|--------|-------|")
            angles = r.hood_current_angle.values
            w(f"| Angle range | {min(angles):.1f} - {max(angles):.1f} deg |")
            if r.hood_current_amps.count > 0:
                w(f"| Current | {r.hood_current_amps.summary(2)} |")
            if r.hood_temp.count > 0:
                w(f"| Temperature | {r.hood_temp.summary(1)} |")
            if r.shot_events:
                hood_ready_shots = sum(1 for s in r.shot_events if s.hood_at_target)
                w(f"| Hood ready at shot time | {hood_ready_shots}/{len(r.shot_events)} shots |")
                hood_not_ready = [s for s in r.shot_events if not s.hood_at_target]
                if hood_not_ready:
                    w(f"| **Shots with hood NOT ready** | **{len(hood_not_ready)}** |")
            w("")
        else:
            w("*No hood data*")
            w("")

        # Turret
        w("## Turret Safety")
        if len(r.turret_absolute_angle) > 0:
            ta = r.turret_absolute_angle.values
            w("| Metric | Value |")
            w("|--------|-------|")
            w(f"| Angle range | {min(ta):.1f} - {max(ta):.1f} deg |")
            if r.turret_current.count > 0:
                w(f"| Current | {r.turret_current.summary(2)} |")
            if len(r.turret_danger_zone) > 0:
                dz_count = sum(1 for v in r.turret_danger_zone.values if v > 0.5)
                w(f"| In danger zone | {dz_count} samples |")
            if len(r.turret_near_limit) > 0:
                nl_count = sum(1 for v in r.turret_near_limit.values if v > 0.5)
                w(f"| Near limit | {nl_count} samples |")
            w("")

            if r.turret_move_events:
                w(f"### Large Turret Moves (>{TURRET_BIG_MOVE_DEG} deg)")
                w(f"**{len(r.turret_move_events)} large moves detected**")
                shots_during = sum(1 for e in r.turret_move_events if e.shot_during)
                if shots_during:
                    w(f"**WARNING: {shots_during} shots fired during large turret moves!**")
                w("")
                w("| Start(s) | End(s) | Change(deg) | Shot During? |")
                w("|----------|--------|-------------|--------------|")
                for e in r.turret_move_events[:20]:
                    w(f"| {ts_sec(e.start_ts, t0):.1f} | {ts_sec(e.end_ts, t0):.1f} | {e.angle_change_deg:.1f} | {'YES' if e.shot_during else 'no'} |")
                w("")
        else:
            w("*No turret data*")
            w("")

        # Spindexer / Recovery correlation
        w("## Spindexer")
        if r.spindexer_current.count > 0:
            w(f"| Metric | Value |")
            w(f"|--------|-------|")
            w(f"| Current | {r.spindexer_current.summary(2)} |")
            if r.spindexer_temp.count > 0:
                w(f"| Temperature | {r.spindexer_temp.summary(1)} |")
            if len(r.spindexer_target_rpm) > 0:
                targets = set(round(v) for v in r.spindexer_target_rpm.values if v != 0)
                w(f"| Target RPMs used | {sorted(targets)} |")
            w("")

        # Motivator
        w("## Motivator")
        if r.motivator_current.count > 0:
            w(f"| Current | {r.motivator_current.summary(2)} |")
            if r.motivator_temp.count > 0:
                w(f"| Temperature | {r.motivator_temp.summary(1)} |")
            w("")

        # Intake
        w("## Intake")
        if len(r.intake_roller_current) > 0:
            roller_vals = r.intake_roller_current.values
            w(f"| Metric | Value |")
            w(f"|--------|-------|")
            w(f"| Roller current mean | {sum(roller_vals)/len(roller_vals):.2f} A |")
            w(f"| Roller current max | {max(roller_vals):.2f} A |")
            if r.intake_roller_rpm.count > 0:
                w(f"| Roller RPM | {r.intake_roller_rpm.summary(1)} |")
            if r.intake_deploy_current.count > 0:
                w(f"| Deploy current | {r.intake_deploy_current.summary(2)} |")
            w("")

        # Fuel flow
        w("## Fuel Flow")
        intake_events = [e for e in r.fuel_events if e.event_type == "intake"]
        w(f"- Intake current spikes (>{INTAKE_CURRENT_SPIKE_A}A): **{len(intake_events)}** detected")
        w(f"- Shots fired: **{len(r.shot_events)}**")
        if len(r.fuel_remaining) > 0:
            fr_vals = r.fuel_remaining.values
            if fr_vals:
                w(f"- Fuel remaining at end: **{int(fr_vals[-1])}**")
                w(f"- Fuel remaining range: {int(min(fr_vals))} - {int(max(fr_vals))}")
        w("")

        # Vision
        w("## Vision")
        total_accepted = 0
        total_rejected = 0
        for cam in range(4):
            acc = sum(r.vision_accepted_count[cam].values) if r.vision_accepted_count[cam].values else 0
            rej = sum(r.vision_rejected_count[cam].values) if r.vision_rejected_count[cam].values else 0
            total_accepted += acc
            total_rejected += rej

        w(f"### Summary: {total_accepted} accepted, {total_rejected} rejected")
        if total_accepted + total_rejected > 0:
            w(f"**Rejection rate: {total_rejected/(total_accepted+total_rejected)*100:.1f}%**")
        w("")

        w("### Per Camera")
        w("| Camera | Observations | Accepted | Rejected | Rej% | Tags Seen |")
        w("|--------|-------------|----------|----------|------|-----------|")
        for cam in range(4):
            obs = r.vision_obs_count[cam]
            acc = sum(r.vision_accepted_count[cam].values) if r.vision_accepted_count[cam].values else 0
            rej = sum(r.vision_rejected_count[cam].values) if r.vision_rejected_count[cam].values else 0
            total = acc + rej
            rej_pct = (rej / total * 100) if total > 0 else 0
            tags = sorted(r.vision_tag_ids[cam].keys()) if r.vision_tag_ids[cam] else []
            tags_str = ', '.join(str(t) for t in tags[:10])
            w(f"| {cam} | {obs.sum_val:.0f} (max={obs.max_val:.0f}) | {acc} | {rej} | {rej_pct:.0f}% | {tags_str} |")
        w("")

        # Tag frequency
        all_tags = {}
        for cam in range(4):
            for tid, cnt in r.vision_tag_ids[cam].items():
                all_tags[tid] = all_tags.get(tid, 0) + cnt
        if all_tags:
            w("### Tag ID Frequency")
            w("| Tag ID | Total Observations |")
            w("|--------|--------------------|")
            for tid, cnt in sorted(all_tags.items(), key=lambda x: -x[1]):
                w(f"| {tid} | {cnt} |")
            w("")

        # Pose jumps
        if r.vision_pose_jumps:
            w(f"### Pose Jumps (>{POSE_JUMP_THRESHOLD_M}m)")
            w(f"**{len(r.vision_pose_jumps)} jumps detected**")
            w("| Time(s) | Distance(m) | Phase |")
            w("|---------|-------------|-------|")
            for ts_us, dist in r.vision_pose_jumps[:20]:
                w(f"| {ts_sec(ts_us, t0):.2f} | {dist:.2f} | {r.phase_at(ts_us)} |")
            w("")

        # Vision connection issues
        for cam in range(4):
            disconnects = [e for e in r.vision_connected[cam] if not e.connected]
            if disconnects:
                w(f"**Camera {cam} disconnected {len(disconnects)} time(s)**")

        w("")

        # Alerts
        w("## Alerts & Errors")
        if r.alert_errors:
            w("### Errors")
            for e in sorted(r.alert_errors):
                w(f"- {e}")
        if r.alert_warnings:
            w("### Warnings")
            for e in sorted(r.alert_warnings):
                w(f"- {e}")
        if r.pathplanner_errors:
            w("### PathPlanner Errors")
            for e in sorted(r.pathplanner_errors):
                w(f"- {e}")
        if r.pathplanner_warnings:
            w("### PathPlanner Warnings")
            for e in sorted(r.pathplanner_warnings):
                w(f"- {e}")
        if r.photon_errors:
            w("### PhotonVision Errors")
            for e in sorted(r.photon_errors):
                w(f"- {e}")
        if r.photon_warnings:
            w("### PhotonVision Warnings")
            for e in sorted(r.photon_warnings):
                w(f"- {e}")
        if r.console_messages:
            w("### Console (last 20)")
            for msg in r.console_messages[-20:]:
                w(f"- `{msg[:200]}`")
        if not any([r.alert_errors, r.alert_warnings, r.pathplanner_errors,
                     r.pathplanner_warnings, r.photon_errors, r.photon_warnings]):
            w("*No alerts*")
        w("")

        # System
        w("## System")
        w("| Metric | Value |")
        w("|--------|-------|")
        if r.cpu_temp.count > 0:
            w(f"| CPU Temp | {r.cpu_temp.summary(1)} |")
        w("")

        # Full field dump
        w("## Full Field Dump")
        w("<details><summary>Click to expand all numeric fields</summary>")
        w("")
        w("| Field | Count | Mean | Min | Max | Std |")
        w("|-------|-------|------|-----|-----|-----|")
        for name, stats in sorted(r.all_fields.items()):
            if stats.count > 0:
                w(f"| `{name}` | {stats.count} | {stats.mean:.3f} | {stats.min_val:.3f} | {stats.max_val:.3f} | {stats.std:.3f} |")
        w("")
        w("</details>")
        w("")

    # ── Cross-file Summary ──
    w("---")
    w("# CROSS-FILE SUMMARY")
    w("")

    # Files overview
    w("## Files Overview")
    w("| File | Size(MB) | Duration(s) | Auto(s) | Teleop(s) | Shots |")
    w("|------|----------|-------------|---------|-----------|-------|")
    for r in reports:
        pd = r.phase_durations()
        w(f"| {r.filename[:40]} | {r.file_size_mb:.0f} | {r.duration_seconds:.0f} | {pd['auto']:.0f} | {pd['teleop']:.0f} | {len(r.shot_events)} |")
    w("")

    # Loop timing comparison
    w("## Loop Timing Comparison")
    w("| File | Mean(ms) | P99(ms) | Max(ms) | Overrun% |")
    w("|------|----------|---------|---------|----------|")
    for r in sorted(reports, key=lambda r: r.full_cycle_ms.percentile(99) if len(r.full_cycle_ms) > 0 else 0, reverse=True):
        if len(r.full_cycle_ms) > 0:
            vals = r.full_cycle_ms.values
            mean = sum(vals) / len(vals)
            overrun = sum(1 for v in vals if v > CYCLE_OVERRUN_MS) / len(vals) * 100
            w(f"| {r.filename[:35]} | {mean:.1f} | {r.full_cycle_ms.percentile(99):.1f} | {max(vals):.1f} | {overrun:.1f}% |")
    w("")

    # Battery comparison
    w("## Battery Health")
    w("| File | Min V | Mean V | Brownouts |")
    w("|------|-------|--------|-----------|")
    for r in reports:
        if len(r.battery_voltage) > 0:
            bv = r.battery_voltage.values
            w(f"| {r.filename[:35]} | {min(bv):.2f} | {sum(bv)/len(bv):.2f} | {len(r.brownout_events)} |")
    w("")

    # Vision rejection rates
    w("## Vision Rejection Rates")
    w("| File | Accepted | Rejected | Rate% |")
    w("|------|----------|----------|-------|")
    for r in sorted(reports, key=lambda r: _rejection_rate(r), reverse=True):
        acc, rej = _total_vision(r)
        total = acc + rej
        rate = (rej / total * 100) if total > 0 else 0
        if total > 0:
            w(f"| {r.filename[:35]} | {acc} | {rej} | {rate:.1f}% |")
    w("")

    # Recovery summary across files
    all_recovery = []
    for r in reports:
        all_recovery.extend(r.recovery_events)
    if all_recovery:
        w("## Launcher Recovery Summary")
        recovery_ms = [e.recovery_time_us / 1000 for e in all_recovery]
        w(f"- Total recovery events: {len(all_recovery)}")
        w(f"- Mean recovery time: {sum(recovery_ms)/len(recovery_ms):.0f} ms")
        w(f"- Max recovery time: {max(recovery_ms):.0f} ms")
        w(f"- P90 recovery time: {sorted(recovery_ms)[int(len(recovery_ms)*0.9)]:.0f} ms")
        w("")

    # Turret safety summary
    all_turret_moves = []
    for r in reports:
        all_turret_moves.extend(r.turret_move_events)
    shots_during_moves = sum(1 for e in all_turret_moves if e.shot_during)
    if all_turret_moves:
        w("## Turret Safety Summary")
        w(f"- Total large moves (>{TURRET_BIG_MOVE_DEG}deg): {len(all_turret_moves)}")
        w(f"- **Shots fired during large moves: {shots_during_moves}**")
        w("")

    # Recurring alerts
    w("## Recurring Alerts")
    all_errors = {}
    all_warnings = {}
    for r in reports:
        for e in r.alert_errors:
            all_errors[e] = all_errors.get(e, 0) + 1
        for w_ in r.alert_warnings:
            all_warnings[w_] = all_warnings.get(w_, 0) + 1
    if all_errors:
        w("### Errors (across files)")
        for e, cnt in sorted(all_errors.items(), key=lambda x: -x[1]):
            w(f"- ({cnt} files) {e}")
    if all_warnings:
        w("### Warnings (across files)")
        for e, cnt in sorted(all_warnings.items(), key=lambda x: -x[1]):
            w(f"- ({cnt} files) {e}")
    w("")

    # Recommendations
    w("## Recommendations")
    _generate_recommendations(reports, w)
    w("")

    # Write
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines))

    print(f"Report written to: {output_path}")
    print(f"Total lines: {len(lines)}")


def _find_t0(reports, filename):
    for r in reports:
        if r.filename == filename:
            return r.first_ts
    return 0


def _total_vision(r):
    acc = sum(sum(r.vision_accepted_count[c].values) for c in range(4) if r.vision_accepted_count[c].values)
    rej = sum(sum(r.vision_rejected_count[c].values) for c in range(4) if r.vision_rejected_count[c].values)
    return acc, rej


def _rejection_rate(r):
    acc, rej = _total_vision(r)
    total = acc + rej
    return (rej / total) if total > 0 else 0


def _generate_recommendations(reports, w):
    """Auto-generate recommendations based on findings."""
    recs = []

    # Loop timing
    overrun_files = []
    for r in reports:
        if len(r.full_cycle_ms) > 0:
            overrun_pct = sum(1 for v in r.full_cycle_ms.values if v > CYCLE_OVERRUN_MS) / len(r.full_cycle_ms) * 100
            if overrun_pct > 5:
                overrun_files.append((r.filename, overrun_pct))
    if overrun_files:
        recs.append(f"**Loop overruns >5%** in {len(overrun_files)} file(s): {', '.join(f[0][:25] for f in overrun_files[:5])}. Check Logger breakdown for bottleneck (AutoLog, ConduitSave, etc.).")

    # Battery
    low_v_files = []
    for r in reports:
        if len(r.battery_voltage) > 0:
            min_v = min(r.battery_voltage.values)
            if min_v < LOW_VOLTAGE_WARN:
                low_v_files.append((r.filename, min_v))
    if low_v_files:
        recs.append(f"**Low battery** in {len(low_v_files)} file(s): min voltage {min(v for _, v in low_v_files):.2f}V. Consider battery management or reducing peak current draw.")

    # Vision
    high_reject = []
    for r in reports:
        rate = _rejection_rate(r)
        if rate > 0.5:
            high_reject.append((r.filename, rate))
    if high_reject:
        recs.append(f"**Vision rejection >50%** in {len(high_reject)} file(s). Check camera calibration, mounting, and filtering parameters.")

    # Turret safety
    all_moves = []
    for r in reports:
        all_moves.extend(r.turret_move_events)
    shots_during = sum(1 for e in all_moves if e.shot_during)
    if shots_during > 0:
        recs.append(f"**{shots_during} shots fired during large turret moves!** Add turret-moving interlock to prevent launching during angle changes >90deg.")

    # Recovery
    all_recovery = []
    for r in reports:
        all_recovery.extend(r.recovery_events)
    if all_recovery:
        slow_recoveries = [e for e in all_recovery if e.recovery_time_us > 500_000]  # >500ms
        if slow_recoveries:
            recs.append(f"**{len(slow_recoveries)} slow launcher recoveries (>500ms).** Consider varying spindexer speed based on launcher RPM target to allow more recovery time at higher speeds.")

    # CAN errors
    can_error_files = []
    for r in reports:
        delta = 0
        if r.can_off_first is not None:
            delta += r.can_off_last - r.can_off_first
        if r.can_rx_err_first is not None:
            delta += r.can_rx_err_last - r.can_rx_err_first
        if r.can_tx_err_first is not None:
            delta += r.can_tx_err_last - r.can_tx_err_first
        if delta > 0:
            can_error_files.append(r.filename)
    if can_error_files:
        recs.append(f"**CAN bus errors** in {len(can_error_files)} file(s). Check wiring and bus load.")

    # Module asymmetry
    for r in reports:
        means = [r.drive_current[i].mean for i in range(4) if r.drive_current[i].count > 0]
        if means:
            avg = sum(means) / len(means)
            if avg > 0:
                for mod in range(4):
                    if r.drive_current[mod].count > 0:
                        pct = abs(r.drive_current[mod].mean - avg) / avg * 100
                        if pct > 30:
                            recs.append(f"**Module {mod} asymmetric current** in {r.filename[:25]} ({pct:.0f}% off average). Check mechanical binding or wheel alignment.")
                            break

    if not recs:
        recs.append("No major issues detected. Nice work!")

    for rec in recs:
        w(f"- {rec}")


# ── Main ────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Analyze WPILog files from FRC robot practice")
    parser.add_argument("directory", help="Directory containing .wpilog files")
    parser.add_argument("-o", "--output", help="Output markdown file path", default=None)
    args = parser.parse_args()

    log_dir = args.directory
    if not os.path.isdir(log_dir):
        print(f"Error: {log_dir} is not a directory")
        sys.exit(1)

    wpilog_files = sorted(Path(log_dir).glob("*.wpilog"))
    if not wpilog_files:
        print(f"No .wpilog files found in {log_dir}")
        sys.exit(1)

    output_path = args.output or os.path.join(log_dir, "analysis_report.md")

    print(f"Found {len(wpilog_files)} .wpilog files")
    print(f"Output: {output_path}")
    print()

    reports = []
    for i, logfile in enumerate(wpilog_files, 1):
        print(f"[{i}/{len(wpilog_files)}] Parsing {logfile.name}...")
        t_start = time.time()
        report = parse_log(str(logfile))
        elapsed = time.time() - t_start
        print(f"  -> {report.record_count:,} records, {report.duration_seconds:.0f}s duration, "
              f"{len(report.shot_events)} shots, parsed in {elapsed:.1f}s")
        reports.append(report)

    print()
    print("Generating report...")
    generate_report(reports, output_path)

    total_shots = sum(len(r.shot_events) for r in reports)
    print(f"Total shots across all files: {total_shots}")
    print("Done!")


if __name__ == "__main__":
    main()
