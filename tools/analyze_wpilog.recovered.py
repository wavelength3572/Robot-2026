# Source Generated with Decompyle++
# File: analyze_wpilog.cpython-312.pyc (Python 3.12)

'''
WPILog Analysis Script for FRC Robot 2026
Parses AdvantageKit .wpilog files and generates a comprehensive Markdown report.

Usage: python analyze_wpilog.py "C:\\path\to\\log\\directory" [-o output.md]
'''
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
CYCLE_OVERRUN_MS = 20
CYCLE_SPIKE_MS = 40
LOW_VOLTAGE_WARN = 11
LOW_VOLTAGE_CRIT = 10.5
POSE3D_SIZE = 56
POSE_JUMP_THRESHOLD_M = 1
TURRET_BIG_MOVE_DEG = 90
TURRET_BIG_MOVE_WINDOW_US = 1000000
LAUNCHER_RECOVERY_DROP_PCT = 0.1
INTAKE_CURRENT_SPIKE_A = 5
RunningStats = <NODE:12>()
TimeSeries = <NODE:12>()
PhaseTransition = <NODE:12>()
ConnectionEvent = <NODE:12>()
ShotEvent = <NODE:12>()
RecoveryEvent = <NODE:12>()
TurretMoveEvent = <NODE:12>()
FuelEvent = <NODE:12>()
LogReport = <NODE:12>()

def ts_sec(ts = dataclass, t0 = dataclass):
    '''Convert microsecond timestamp to seconds relative to t0.'''
    return (ts - t0) / 1e+06


def parse_log(filepath = dataclass):
    '''Parse a single .wpilog file and return a LogReport.'''
    pass
# WARNING: Decompyle incomplete


def _get_handler(name = dataclass, dtype = dataclass, report = dataclass):
    """Return a handler closure for the given field, or None if we don't care about it."""
    pass
# WARNING: Decompyle incomplete


def _detect_recovery_events(report = None):
    '''Detect launcher recovery events from velocity time series.'''
    vel = report.launcher_velocity
    tgt = report.launcher_target_rpm
    if len(vel) < 10 or len(tgt) < 1:
        return None
    tgt_vals = list(zip(tgt.timestamps, tgt.values))
    tgt_idx = 0
    in_recovery = False
    recovery_start_ts = 0
    recovery_drop = 0
    recovery_target = 0
    for i in range(len(vel)):
        ts = vel.timestamps[i]
        v = vel.values[i]
        if tgt_idx < len(tgt_vals) - 1 and tgt_vals[tgt_idx + 1][0] <= ts:
            tgt_idx += 1
            if tgt_idx < len(tgt_vals) - 1 and tgt_vals[tgt_idx + 1][0] <= ts:
                continue
        target = tgt_vals[tgt_idx][1] if tgt_idx < len(tgt_vals) else 0
        if target <= 100:
            in_recovery = False
            continue
        drop_pct = (target - v) / target if target > 0 else 0
        if in_recovery and drop_pct > LAUNCHER_RECOVERY_DROP_PCT:
            in_recovery = True
            recovery_start_ts = ts
            recovery_drop = target - v
            recovery_target = target
            continue
        if not in_recovery:
            continue
        if not drop_pct <= 0.05:
            continue
        spindexer_rpm = 0
        if report.spindexer_rpm.timestamps:
            for j in range(len(report.spindexer_rpm.timestamps)):
                if not report.spindexer_rpm.timestamps[j] >= recovery_start_ts:
                    continue
                spindexer_rpm = report.spindexer_rpm.values[j]
                range(len(report.spindexer_rpm.timestamps))
        report.recovery_events.append(RecoveryEvent(timestamp_us = recovery_start_ts, drop_rpm = recovery_drop, recovery_time_us = ts - recovery_start_ts, spindexer_rpm = spindexer_rpm, target_rpm = recovery_target))
        in_recovery = False


def _detect_turret_moves(report = None):
    '''Detect large turret angle changes.'''
    angle = report.turret_absolute_angle
    if len(angle) < 10:
        return None
    j = 0
    for i in range(len(angle)):
        if j < len(angle) and angle.timestamps[j] - angle.timestamps[i] < TURRET_BIG_MOVE_WINDOW_US:
            j += 1
            if j < len(angle) and angle.timestamps[j] - angle.timestamps[i] < TURRET_BIG_MOVE_WINDOW_US:
                continue
        if j >= len(angle):
            range(len(angle))
            return None
        for k in range(i, min(j, len(angle))):
            change = abs(angle.values[k] - angle.values[i])
            if not change > TURRET_BIG_MOVE_DEG:
                continue
            shot_during = False
            for evt in report.shot_events:
                if  <= angle.timestamps[i], evt.timestamp_us:
                    if not angle.timestamps[i], evt.timestamp_us <= angle.timestamps[min(k, len(angle) - 1)]:
                        continue
                    else:
                        report.shot_events
                    True
                
            report.turret_move_events.append(TurretMoveEvent(start_ts = angle.timestamps[i], end_ts = angle.timestamps[k], angle_change_deg = change, shot_during = shot_during))


def _detect_fuel_events(report = None):
    '''Detect fuel intake events from roller current spikes.'''
    roller = report.intake_roller_current
    deployed = report.intake_deployed
    if len(roller) < 10:
        return None
    deployed_at = { }
    if deployed.timestamps:
        for i in range(len(deployed)):
            deployed_at[deployed.timestamps[i]] = deployed.values[i] > 0.5
    was_spiking = False
    for i in range(len(roller)):
        ts = roller.timestamps[i]
        val = roller.values[i]
        if not val > INTAKE_CURRENT_SPIKE_A and was_spiking:
            report.fuel_events.append(FuelEvent(ts, 'intake', val))
            was_spiking = True
            continue
        if not val <= INTAKE_CURRENT_SPIKE_A * 0.7:
            continue
        was_spiking = False


def generate_report(reports = None, output_path = None):
    '''Generate comprehensive Markdown report.'''
    lines = []
    w = lines.append
    w('# WPILog Analysis Report - GVSU Practice 2/28')
    w(f'''*Generated: {time.strftime('%Y-%m-%d %H:%M:%S')}*''')
    w(f'''*Files analyzed: {len(reports)}*''')
    w('')
    all_shots = []
    for r in reports:
        all_shots.extend(r.shot_events)
    if all_shots:
        w('---')
        w('# SHOT DATA (Lookup Table Seed)')
        w('')
        w('| # | File | Phase | Time(s) | Dist(m) | Hood(deg) | IdealRPM | ActualRPM | VelErr | ExitVel(m/s) | LaunchAng(deg) | Azimuth(deg) | TurretAbs(deg) | HoodReady | LauncherReady | SpindexerRPM | FuelLeft |')
        w('|---|------|-------|---------|---------|-----------|----------|-----------|--------|--------------|----------------|--------------|----------------|-----------|---------------|--------------|----------|')
        for i, s in enumerate(all_shots, 1):
            t = ts_sec(s.timestamp_us, _find_t0(reports, s.file_name))
            []['| '][f'''{i}'''][' | '][f'''{s.file_name[:25]}'''][' | '][f'''{s.phase}'''][' | '][f'''{t:.1f}'''][' | '][f'''{s.distance_m:.2f}'''][' | '][f'''{s.hood_angle_deg:.1f}'''][' | '][f'''{s.ideal_rpm:.0f}'''][' | '][f'''{s.current_rpm:.0f}'''][' | '][f'''{s.velocity_error:.0f}'''][' | '][f'''{s.exit_velocity_mps:.1f}'''][' | '][f'''{s.launch_angle_deg:.1f}'''][' | '][f'''{s.azimuth_deg:.1f}'''][' | '][f'''{s.turret_absolute_deg:.1f}'''][' | '][f'''{'Y' if s.hood_at_target else 'N'}'''][' | '][f'''{'Y' if s.launcher_at_setpoint else 'N'}'''][' | '][f'''{s.spindexer_rpm:.0f}'''][' | ']([]['| '][f'''{i}'''][' | '][f'''{s.file_name[:25]}'''][' | '][f'''{s.phase}'''][' | '][f'''{t:.1f}'''][' | '][f'''{s.distance_m:.2f}'''][' | '][f'''{s.hood_angle_deg:.1f}'''][' | '][f'''{s.ideal_rpm:.0f}'''][' | '][f'''{s.current_rpm:.0f}'''][' | '][f'''{s.velocity_error:.0f}'''][' | '][f'''{s.exit_velocity_mps:.1f}'''][' | '][f'''{s.launch_angle_deg:.1f}'''][' | '][f'''{s.azimuth_deg:.1f}'''][' | '][f'''{s.turret_absolute_deg:.1f}'''][' | '][f'''{'Y' if s.hood_at_target else 'N'}'''][' | '][f'''{'Y' if s.launcher_at_setpoint else 'N'}'''][' | '][f'''{s.spindexer_rpm:.0f}'''][' | '][f'''{s.fuel_remaining}''']([]['| '][f'''{i}'''][' | '][f'''{s.file_name[:25]}'''][' | '][f'''{s.phase}'''][' | '][f'''{t:.1f}'''][' | '][f'''{s.distance_m:.2f}'''][' | '][f'''{s.hood_angle_deg:.1f}'''][' | '][f'''{s.ideal_rpm:.0f}'''][' | '][f'''{s.current_rpm:.0f}'''][' | '][f'''{s.velocity_error:.0f}'''][' | '][f'''{s.exit_velocity_mps:.1f}'''][' | '][f'''{s.launch_angle_deg:.1f}'''][' | '][f'''{s.azimuth_deg:.1f}'''][' | '][f'''{s.turret_absolute_deg:.1f}'''][' | '][f'''{'Y' if s.hood_at_target else 'N'}'''][' | '][f'''{'Y' if s.launcher_at_setpoint else 'N'}'''][' | '][f'''{s.spindexer_rpm:.0f}'''][' | '][f'''{s.fuel_remaining}'''][' |']))
        w('')
        not_ready_hood = (lambda .0: pass# WARNING: Decompyle incomplete
)(all_shots())
        not_ready_launcher = (lambda .0: pass# WARNING: Decompyle incomplete
)(all_shots())
        w(f'''**Total shots: {len(all_shots)}** | Hood not ready: {not_ready_hood} | Launcher not ready: {not_ready_launcher}''')
        w('')
# WARNING: Decompyle incomplete


def _find_t0(reports, filename):
    for r in reports:
        if not r.filename == filename:
            continue
        
        return reports, r.first_ts
    return 0


def _total_vision(r):
    pass
# WARNING: Decompyle incomplete


def _rejection_rate(r):
    (acc, rej) = _total_vision(r)
    total = acc + rej
    if total > 0:
        return rej / total


def _generate_recommendations(reports, w):
    '''Auto-generate recommendations based on findings.'''
    recs = []
    overrun_files = []
    for r in reports:
        if not len(r.full_cycle_ms) > 0:
            continue
        overrun_pct = ((lambda .0: pass# WARNING: Decompyle incomplete
)(r.full_cycle_ms.values()) / len(r.full_cycle_ms)) * 100
        if not overrun_pct > 5:
            continue
        overrun_files.append((r.filename, overrun_pct))
    if overrun_files:
        f'''{len(overrun_files)}'''(f''' file(s): {(lambda .0: pass# WARNING: Decompyle incomplete
)(overrun_files[:5]())}. Check Logger breakdown for bottleneck (AutoLog, ConduitSave, etc.).''')
    low_v_files = []
    for r in reports:
        if not len(r.battery_voltage) > 0:
            continue
        min_v = min(r.battery_voltage.values)
        if not min_v < LOW_VOLTAGE_WARN:
            continue
        low_v_files.append((r.filename, min_v))
    if low_v_files:
        f'''{len(low_v_files)}'''(f''' file(s): min voltage {(lambda .0: pass# WARNING: Decompyle incomplete
)(low_v_files()):.2f}V. Consider battery management or reducing peak current draw.''')
    high_reject = []
    for r in reports:
        rate = _rejection_rate(r)
        if not rate > 0.5:
            continue
        high_reject.append((r.filename, rate))
    if high_reject:
        recs.append(f'''**Vision rejection >50%** in {len(high_reject)} file(s). Check camera calibration, mounting, and filtering parameters.''')
    all_moves = []
    for r in reports:
        all_moves.extend(r.turret_move_events)
    shots_during = (lambda .0: pass# WARNING: Decompyle incomplete
)(all_moves())
    if shots_during > 0:
        recs.append(f'''**{shots_during} shots fired during large turret moves!** Add turret-moving interlock to prevent launching during angle changes >90deg.''')
    all_recovery = []
    for r in reports:
        all_recovery.extend(r.recovery_events)
# WARNING: Decompyle incomplete


def main():
    parser = argparse.ArgumentParser(description = 'Analyze WPILog files from FRC robot practice')
    parser.add_argument('directory', help = 'Directory containing .wpilog files')
    parser.add_argument('-o', '--output', help = 'Output markdown file path', default = None)
    args = parser.parse_args()
    log_dir = args.directory
    if not os.path.isdir(log_dir):
        print(f'''Error: {log_dir} is not a directory''')
        sys.exit(1)
    wpilog_files = sorted(Path(log_dir).glob('*.wpilog'))
    if not wpilog_files:
        print(f'''No .wpilog files found in {log_dir}''')
        sys.exit(1)
    if not args.output:
        args.output
    output_path = os.path.join(log_dir, 'analysis_report.md')
    print(f'''Found {len(wpilog_files)} .wpilog files''')
    print(f'''Output: {output_path}''')
    print()
    reports = []
    for i, logfile in enumerate(wpilog_files, 1):
        print(f'''[{i}/{len(wpilog_files)}] Parsing {logfile.name}...''')
        t_start = time.time()
        report = parse_log(str(logfile))
        elapsed = time.time() - t_start
        print(f'''  -> {report.record_count:,} records, {report.duration_seconds:.0f}s duration, {len(report.shot_events)} shots, parsed in {elapsed:.1f}s''')
        reports.append(report)
    print()
    print('Generating report...')
    generate_report(reports, output_path)
    total_shots = (lambda .0: pass# WARNING: Decompyle incomplete
)(reports())
    print(f'''Total shots across all files: {total_shots}''')
    print('Done!')

if __name__ == '__main__':
    main()
    return None
