package frc.robot.subsystems.shooting;

/**
 * A single recorded shot data point. Stores both COMMANDED (target) and ACTUAL (measured) values.
 *
 * <p>The LUT uses the commanded/target values (what settings produce good shots). The actual values
 * are diagnostic — useful for offline analysis of recovery time, RPM consistency, efficiency
 * validation, and feed speed tuning.
 *
 * <p><b>LUT fields:</b> distanceM, targetRPM, targetHoodAngleDeg, timeOfFlightS, successful
 *
 * <p><b>Diagnostic fields:</b> actualRPM, actualHoodAngleDeg, actualMotivatorRPM,
 * actualSpindexerRPM, targetMotivatorRPM, targetSpindexerRPM
 *
 * @param timestamp FPGA timestamp when the shot was recorded
 * @param robotX Robot X position on field (meters)
 * @param robotY Robot Y position on field (meters)
 * @param distanceM Horizontal distance from turret to target (meters)
 * @param targetRPM Commanded launcher RPM setpoint (what the LUT stores)
 * @param targetHoodAngleDeg Commanded hood angle from shot calculation (degrees)
 * @param turretAngleDeg Actual turret angle (degrees, robot-relative)
 * @param targetMotivatorRPM Commanded motivator RPM setpoint
 * @param targetSpindexerRPM Commanded spindexer RPM setpoint
 * @param actualRPM Actual launcher wheel RPM at time of recording (diagnostic)
 * @param actualHoodAngleDeg Actual hood position at time of recording (diagnostic)
 * @param actualMotivatorRPM Actual motivator RPM at time of recording (diagnostic)
 * @param actualSpindexerRPM Actual spindexer RPM at time of recording (diagnostic)
 * @param exitVelocityMps Calculated exit velocity based on target RPM (m/s)
 * @param timeOfFlightS Calculated time of flight (seconds)
 * @param successful Whether all shots scored
 */
public record ShotDataPoint(
    double timestamp,
    double robotX,
    double robotY,
    double distanceM,
    // Target/commanded values (used by LUT)
    double targetRPM,
    double targetHoodAngleDeg,
    double turretAngleDeg,
    double targetMotivatorRPM,
    double targetSpindexerRPM,
    // Actual/measured values (diagnostic)
    double actualRPM,
    double actualHoodAngleDeg,
    double actualMotivatorRPM,
    double actualSpindexerRPM,
    // Derived values
    double exitVelocityMps,
    double timeOfFlightS,
    boolean successful) {}
