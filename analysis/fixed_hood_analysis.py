"""
Fixed Hood Angle Analysis
=========================
Compares variable hood (current design) vs fixed hood (proposed) using
the exact physics and constants from TrajectoryOptimizer.java and TurretCalculator.java.

Question: If we fix the hood angle and only vary RPM, can we still make shots
from 60" to 208" from the hub? And what do we lose?
"""

import math

# ─── Constants from codebase ───
GRAVITY = 9.81  # m/s²
HUB_LIP_HEIGHT = 1.83  # meters (TrajectoryOptimizer.java:28)
HUB_CENTER_HEIGHT = 1.43  # meters (TrajectoryOptimizer.java:29)
HUB_ENTRY_RADIUS = 0.530  # meters (TrajectoryOptimizer.java:31)
TURRET_HEIGHT = 0.3597  # meters (TurretConstants.java)
WHEEL_RADIUS = 0.0508  # meters (TurretCalculator.java:23)
LAUNCH_EFFICIENCY = 0.70  # (TurretCalculator.java:27)
MIN_RPM = 1500.0
MAX_RPM = 4000.0
MIN_EXIT_VELOCITY = 3.0  # m/s
MAX_EXIT_VELOCITY = 15.0  # m/s
DESIRED_DESCENT_ANGLE_DEG = 60.0  # current tuned value

# Distance range (inches, converted to meters)
DISTANCES_INCHES = list(range(60, 220, 12))  # 60" to 208" in 12" steps
DISTANCES_METERS = [d * 0.0254 for d in DISTANCES_INCHES]


def rpm_to_velocity(rpm):
    """RPM -> exit velocity (m/s). From TurretCalculator.java:114-119"""
    surface_vel = (rpm * 2.0 * math.pi * WHEEL_RADIUS) / 60.0
    return surface_vel * LAUNCH_EFFICIENCY


def velocity_to_rpm(vel):
    """Exit velocity (m/s) -> RPM. From TurretCalculator.java:128-131"""
    surface_vel = vel / LAUNCH_EFFICIENCY
    return (surface_vel * 60.0) / (2.0 * math.pi * WHEEL_RADIUS)


def analyze_variable_hood(D_hub_center):
    """
    Current system: Variable hood + variable RPM.
    Reproduces TrajectoryOptimizer.calculateOptimalShot logic.
    Two degrees of freedom -> trajectory passes through hub edge AND hub center
    with the desired 60° descent angle.
    """
    D_edge = D_hub_center - HUB_ENTRY_RADIUS

    # Descent angle determines height at hub edge
    descent_rad = math.radians(DESIRED_DESCENT_ANGLE_DEG)
    height_drop = HUB_ENTRY_RADIUS * math.tan(descent_rad)
    height_at_edge = HUB_CENTER_HEIGHT + height_drop
    clearance_m = height_at_edge - HUB_LIP_HEIGHT
    clearance_in = clearance_m / 0.0254

    # Heights relative to turret
    H_edge = height_at_edge - TURRET_HEIGHT  # y1
    H_target = HUB_CENTER_HEIGHT - TURRET_HEIGHT  # y2
    x1 = D_edge
    x2 = D_hub_center
    y1 = H_edge
    y2 = H_target

    # Solve for launch angle (from TrajectoryOptimizer.java:196-203)
    denom = x1 * x2 * (x2 - x1)
    if abs(denom) < 0.001:
        return None
    tan_theta = (y1 * x2**2 - y2 * x1**2) / denom
    theta = math.atan(tan_theta)
    theta_deg = math.degrees(theta)

    # Solve for K and velocity
    K = (x1 * tan_theta - y1) / (x1**2)
    if K <= 0:
        return None
    cos_theta = math.cos(theta)
    v_squared = GRAVITY / (2 * K * cos_theta**2)
    if v_squared <= 0:
        return None
    velocity = math.sqrt(v_squared)
    rpm = velocity_to_rpm(velocity)

    # Peak height
    vy0 = velocity * math.sin(theta)
    peak_height = TURRET_HEIGHT + (vy0**2) / (2 * GRAVITY)

    # Check peak before hub edge
    vx = velocity * cos_theta
    time_to_peak = vy0 / GRAVITY
    dist_to_peak = vx * time_to_peak

    achievable = (15.0 <= theta_deg <= 85.0 and
                  MIN_RPM <= rpm <= MAX_RPM and
                  dist_to_peak < x1 and
                  peak_height <= 4.0)

    return {
        'theta_deg': theta_deg,
        'rpm': rpm,
        'velocity': velocity,
        'peak_height': peak_height,
        'descent_angle': DESIRED_DESCENT_ANGLE_DEG,
        'clearance_in': clearance_in,
        'achievable': achievable
    }


def analyze_fixed_hood(D_hub_center, fixed_angle_deg):
    """
    Proposed system: Fixed hood angle + variable RPM only.
    One degree of freedom -> can hit hub center point, but descent angle
    and lip clearance are whatever they happen to be.
    """
    theta = math.radians(fixed_angle_deg)
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)
    tan_theta = math.tan(theta)

    # Target: hub center at (D_hub_center, HUB_CENTER_HEIGHT - TURRET_HEIGHT)
    x_target = D_hub_center
    y_target = HUB_CENTER_HEIGHT - TURRET_HEIGHT

    # Projectile: y = x*tan(θ) - g*x²/(2*v²*cos²(θ))
    # Solve for v²: v² = g*x²/(2*cos²(θ)*(x*tan(θ) - y))
    numerator_check = x_target * tan_theta - y_target
    if numerator_check <= 0:
        return None  # Can't reach target with this angle

    v_squared = (GRAVITY * x_target**2) / (2 * cos_theta**2 * numerator_check)
    velocity = math.sqrt(v_squared)
    rpm = velocity_to_rpm(velocity)

    # Check velocity/RPM limits
    if velocity < MIN_EXIT_VELOCITY or velocity > MAX_EXIT_VELOCITY:
        rpm_ok = False
    elif rpm < MIN_RPM or rpm > MAX_RPM:
        rpm_ok = False
    else:
        rpm_ok = True

    # Peak height
    vy0 = velocity * sin_theta
    peak_height = TURRET_HEIGHT + (vy0**2) / (2 * GRAVITY)

    # Time to peak
    vx = velocity * cos_theta
    time_to_peak = vy0 / GRAVITY
    dist_to_peak = vx * time_to_peak

    # Height at hub edge (D - R from turret)
    D_edge = D_hub_center - HUB_ENTRY_RADIUS
    height_at_edge_rel = D_edge * tan_theta - (GRAVITY * D_edge**2) / (2 * v_squared * cos_theta**2)
    height_at_edge_abs = height_at_edge_rel + TURRET_HEIGHT
    clearance_m = height_at_edge_abs - HUB_LIP_HEIGHT
    clearance_in = clearance_m / 0.0254

    # Actual descent angle: velocity components at hub edge
    t_edge = D_edge / vx
    vy_at_edge = vy0 - GRAVITY * t_edge
    # Descent angle = angle of velocity vector below horizontal at hub edge
    if vy_at_edge >= 0:
        # Ball still rising at edge -> BAD
        actual_descent_deg = 0  # Not descending
        ball_descending = False
    else:
        actual_descent_deg = math.degrees(math.atan(-vy_at_edge / vx))
        ball_descending = True

    # Velocity components at hub center
    t_center = D_hub_center / vx
    vy_at_center = vy0 - GRAVITY * t_center
    if vy_at_center >= 0:
        center_descent_deg = 0
    else:
        center_descent_deg = math.degrees(math.atan(-vy_at_center / vx))

    # Check if ball clears lip
    clears_lip = clearance_in >= 2.0  # minimum clearance

    # Check peak is before edge (ball descending at entry)
    descending_at_edge = dist_to_peak < D_edge

    achievable = rpm_ok and clears_lip and descending_at_edge and peak_height <= 4.0

    return {
        'theta_deg': fixed_angle_deg,
        'rpm': rpm,
        'velocity': velocity,
        'peak_height': peak_height,
        'height_at_edge': height_at_edge_abs,
        'clearance_in': clearance_in,
        'descent_at_edge_deg': actual_descent_deg,
        'descent_at_center_deg': center_descent_deg,
        'ball_descending_at_edge': ball_descending,
        'dist_to_peak': dist_to_peak,
        'D_edge': D_edge,
        'achievable': achievable,
        'rpm_ok': rpm_ok,
        'clears_lip': clears_lip,
    }


def hub_margin_analysis(result):
    """
    Calculate effective margin for a shot entering the hub.
    The hub opening is ~20.865" diameter. A steeper descent angle
    means the ball drops more vertically into the opening,
    giving more horizontal margin for error.

    At descent angle α, effective horizontal margin ≈ R * sin(α)
    where R is the hub entry radius.
    """
    if result is None:
        return 0
    descent = result.get('descent_at_center_deg', result.get('descent_angle', 0))
    if descent <= 0:
        return 0
    # Effective width of the hub opening as seen by the incoming ball
    effective_opening = HUB_ENTRY_RADIUS * 2 * math.sin(math.radians(descent))
    return effective_opening


def print_analysis():
    print("=" * 100)
    print("FIXED HOOD ANGLE ANALYSIS")
    print("Using constants from TrajectoryOptimizer.java and TurretCalculator.java")
    print("=" * 100)
    print()

    # Test several candidate fixed angles
    candidate_angles = [35, 40, 45, 50, 55, 60, 65, 70, 75]

    # ─── Part 1: Variable hood baseline ───
    print("─" * 100)
    print("BASELINE: Variable Hood (current design) - 60° descent angle target")
    print("─" * 100)
    print(f"{'Dist(in)':>8} {'Dist(m)':>8} {'Hood°':>7} {'RPM':>7} {'Vel(m/s)':>9} "
          f"{'Peak(m)':>8} {'Clear(in)':>10} {'Descent°':>9} {'Margin(in)':>11} {'OK':>4}")

    baseline_margins = {}
    for d_in, d_m in zip(DISTANCES_INCHES, DISTANCES_METERS):
        r = analyze_variable_hood(d_m)
        if r:
            margin = hub_margin_analysis(r) / 0.0254  # convert to inches
            baseline_margins[d_in] = margin
            print(f"{d_in:>8} {d_m:>8.3f} {r['theta_deg']:>7.1f} {r['rpm']:>7.0f} "
                  f"{r['velocity']:>9.2f} {r['peak_height']:>8.2f} {r['clearance_in']:>10.1f} "
                  f"{r['descent_angle']:>9.1f} {margin:>11.1f} {'YES' if r['achievable'] else 'NO':>4}")
        else:
            print(f"{d_in:>8} {d_m:>8.3f} {'--- UNREACHABLE ---':>60}")

    # ─── Part 2: Fixed hood at each candidate angle ───
    for fixed_angle in candidate_angles:
        print()
        print("─" * 100)
        print(f"FIXED HOOD: {fixed_angle}° (only varying RPM)")
        print("─" * 100)
        print(f"{'Dist(in)':>8} {'Dist(m)':>8} {'RPM':>7} {'Vel(m/s)':>9} "
              f"{'Peak(m)':>8} {'Clear(in)':>10} {'Desc@Edge°':>11} {'Desc@Ctr°':>10} "
              f"{'Margin(in)':>11} {'Δ Margin':>9} {'OK':>4} {'Notes':>20}")

        achievable_count = 0
        for d_in, d_m in zip(DISTANCES_INCHES, DISTANCES_METERS):
            r = analyze_fixed_hood(d_m, fixed_angle)
            if r:
                margin = hub_margin_analysis(r) / 0.0254
                baseline_m = baseline_margins.get(d_in, 0)
                delta_margin = margin - baseline_m

                notes = ""
                if not r['rpm_ok']:
                    notes = f"RPM={r['rpm']:.0f} OOB"
                elif not r['clears_lip']:
                    notes = f"Clip lip {r['clearance_in']:.1f}in"
                elif not r['ball_descending_at_edge']:
                    notes = "Still rising"
                elif r['achievable']:
                    achievable_count += 1

                print(f"{d_in:>8} {d_m:>8.3f} {r['rpm']:>7.0f} "
                      f"{r['velocity']:>9.2f} {r['peak_height']:>8.2f} {r['clearance_in']:>10.1f} "
                      f"{r['descent_at_edge_deg']:>11.1f} {r['descent_at_center_deg']:>10.1f} "
                      f"{margin:>11.1f} {delta_margin:>+9.1f} "
                      f"{'YES' if r['achievable'] else 'NO':>4} {notes:>20}")
            else:
                print(f"{d_in:>8} {d_m:>8.3f} {'--- UNREACHABLE (angle too low for distance) ---':>70}")

        total = len(DISTANCES_INCHES)
        print(f"\n  -> {achievable_count}/{total} distances achievable with fixed {fixed_angle}° hood")

    # ─── Part 3: Shooting-on-the-move analysis ───
    print()
    print("=" * 100)
    print("SHOOTING ON THE MOVE ANALYSIS")
    print("=" * 100)
    print()
    print("With variable hood (2 DOF: angle + RPM):")
    print("  - Can independently adjust trajectory shape AND speed")
    print("  - Moving shot compensation: predict target position, recalculate both angle and RPM")
    print("  - The trajectory still passes through hub edge and center at desired descent angle")
    print("  - Robot lateral motion is compensated by turret azimuth tracking")
    print()
    print("With fixed hood (1 DOF: RPM only):")
    print("  - Can only adjust speed, not trajectory shape")
    print("  - Moving shot changes effective distance -> RPM adjusts -> descent angle changes")
    print("  - At distances where descent angle is already marginal, motion makes it worse")
    print()

    # Simulate: robot moving at 2 m/s laterally, how much does effective distance change?
    robot_speed = 2.0  # m/s (typical driving speed)
    print(f"Example: Robot moving at {robot_speed} m/s perpendicular to hub")
    print(f"{'Dist(in)':>8} {'ToF(s)':>8} {'Δ Dist(in)':>11} {'Δ Descent°':>11}")

    best_angle = 45  # Use 45° as example
    for d_in, d_m in zip(DISTANCES_INCHES, DISTANCES_METERS):
        r_static = analyze_fixed_hood(d_m, best_angle)
        if r_static and r_static['achievable']:
            # Time of flight
            vx = r_static['velocity'] * math.cos(math.radians(best_angle))
            tof = d_m / vx

            # During flight, robot moves -> effective distance changes
            lateral_shift = robot_speed * tof
            new_dist = math.sqrt(d_m**2 + lateral_shift**2)
            delta_dist_in = (new_dist - d_m) / 0.0254

            r_moved = analyze_fixed_hood(new_dist, best_angle)
            if r_moved:
                delta_descent = r_moved['descent_at_center_deg'] - r_static['descent_at_center_deg']
                print(f"{d_in:>8} {tof:>8.3f} {delta_dist_in:>+11.1f} {delta_descent:>+11.1f}")

    # ─── Part 4: Summary ───
    print()
    print("=" * 100)
    print("SUMMARY")
    print("=" * 100)
    print()
    print("SIDE 1 (Pro fixed hood) is correct that:")
    print("  - A fixed hood CAN make shots across the 60-208 inch range")
    print("  - Build complexity is significantly reduced (no hood motor/encoder/zeroing)")
    print("  - Varying RPM alone is sufficient to HIT the hub center")
    print()
    print("SIDE 2 (Against fixed hood) is correct that:")
    print("  - The trajectory will NOT go through hub edge AND center at the ideal descent angle")
    print("  - With 1 DOF you can aim at the center, but you lose control of the entry angle")
    print("  - At some distances, descent angle is shallow -> smaller effective opening")
    print("  - Shooting on the move compounds this: distance changes during flight shift the")
    print("    descent angle further from ideal, reducing margin that's already reduced")
    print()
    print("KEY FINDING:")
    print("  The variable hood system has 2 DOF (angle + RPM) which lets it define a UNIQUE")
    print("  parabola through 2 points (hub edge at controlled height + hub center).")
    print("  A fixed hood has 1 DOF (RPM only) which lets it hit 1 point (hub center) but")
    print("  the entry geometry is dictated by physics, not by design choice.")
    print()
    print("  See the 'Margin(in)' and 'Δ Margin' columns above for quantitative impact.")


if __name__ == "__main__":
    print_analysis()
