package frc.robot.subsystems.intake;

public final class IntakeConstants {
  // Hardware CAN IDs (placeholders - update when known)
  public static final int DEPLOY_MOTOR_CAN_ID = 100;
  public static final int ROLLER_MOTOR_CAN_ID = 20;

  // Deploy Motor Configuration
  public static final double DEPLOY_GEAR_RATIO = 25.0; // Motor rotations per mechanism rotation
  public static final boolean DEPLOY_MOTOR_INVERTED = false;

  // Deploy Position Limits (in mechanism rotations)
  public static final double DEPLOY_RETRACTED_POSITION = 0.0; // Home/retracted position
  public static final double DEPLOY_EXTENDED_POSITION = 0.5; // Fully deployed position

  // Deploy PID Constants (position control)
  public static final double DEPLOY_KP = 5.0;
  public static final double DEPLOY_KI = 0.0;
  public static final double DEPLOY_KD = 0.1;

  // Deploy Tolerances
  public static final double DEPLOY_POSITION_TOLERANCE = 0.02; // rotations

  // Deploy Current Limit
  public static final int DEPLOY_CURRENT_LIMIT = 30; // Amps

  // Roller Motor Configuration
  public static final double ROLLER_GEAR_RATIO = 1.0; // Motor rotations per roller rotation
  public static final boolean ROLLER_MOTOR_INVERTED = false;

  // Roller Speed Constants (duty cycle -1 to 1)
  public static final double ROLLER_INTAKE_SPEED = 0.8;
  public static final double ROLLER_EJECT_SPEED = -0.6;
  public static final double ROLLER_HOLD_SPEED = 0.1;

  // Roller Velocity PID Constants (closed-loop velocity control)
  public static final double ROLLER_KP = 0.0001;
  public static final double ROLLER_KI = 0.0;
  public static final double ROLLER_KD = 0.000001;
  public static final double ROLLER_KFF = 0.000225;

  // Roller Velocity Targets (RPM at mechanism output)
  public static final double ROLLER_INTAKE_RPM = 1500.0;
  public static final double ROLLER_EJECT_RPM = -1000.0;
  public static final double ROLLER_HOLD_RPM = 200.0;

  // Roller Current Limit
  public static final int ROLLER_CURRENT_LIMIT = 90; // Amps

  // Simulation Constants
  public static final double DEPLOY_SIM_MOI = 0.01; // kg*m^2 moment of inertia
  public static final double ROLLER_SIM_MOI = 0.001; // kg*m^2 moment of inertia
}
