package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

// TODO might need some sort of deploy sequence or intelligence to fix the backlash with respect to
// deploy

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  // Tunable PID gains for deploy motor
  private static final LoggedTunableNumber deployKP;
  private static final LoggedTunableNumber deployKI;
  private static final LoggedTunableNumber deployKD;

  // Tunable PID gains for roller velocity control
  private static final LoggedTunableNumber rollerKP;
  private static final LoggedTunableNumber rollerKI;
  private static final LoggedTunableNumber rollerKD;
  private static final LoggedTunableNumber rollerKFF;

  // Tunable deploy positions (adjustable live for testing)
  private static final LoggedTunableNumber deployStowedPos;
  private static final LoggedTunableNumber deployExtendedPos;
  private static final LoggedTunableNumber deployRetractedPos;
  private static final LoggedTunableNumber deployTolerance;
  private static final LoggedTunableNumber holdTolerance;

  // Tunable feedforward gains for deploy MAXMotion
  private static final LoggedTunableNumber deployKS;
  private static final LoggedTunableNumber deployKV;

  // Tunable MAXMotion parameters — separate sets for deploy vs retract
  private static final LoggedTunableNumber deployMaxVelocity;
  private static final LoggedTunableNumber deployMaxAcceleration;
  private static final LoggedTunableNumber retractMaxVelocity;
  private static final LoggedTunableNumber retractMaxAcceleration;

  // Output range safety limits (caps deploy PID duty cycle)
  private static final LoggedTunableNumber deployOutputLimit;
  private static final LoggedTunableNumber retractOutputLimit;

  // Minimum deploy position for rollers to run (below this, rollers are blocked)
  private static final LoggedTunableNumber rollerMinDeployPosition;

  // How long to brake after deploy reaches target before switching to coast
  private static final LoggedTunableNumber deployBrakeTime;

  // Agitation tunables
  private static final LoggedTunableNumber agitationFallTime;
  private static final LoggedTunableNumber agitationSpeedThreshold;
  private static final LoggedTunableNumber agitationRetractTarget;
  private static final LoggedTunableNumber agitationTimeoutSec;
  private static final LoggedTunableNumber agitationCoastTimeSec;
  private static final LoggedTunableNumber agitationRetractOutputLimit;
  private static final LoggedTunableNumber agitationMaxVelocity;
  private static final LoggedTunableNumber agitationMaxAcceleration;
  private static final LoggedTunableNumber agitationStationaryDwellSec;

  static {
    RobotConfig config = Constants.getRobotConfig();
    deployKP = new LoggedTunableNumber("Tuning/Intake/IntakeDeploy/kP", config.getIntakeDeployKp());
    deployKI = new LoggedTunableNumber("Tuning/Intake/IntakeDeploy/kI", config.getIntakeDeployKi());
    deployKD = new LoggedTunableNumber("Tuning/Intake/IntakeDeploy/kD", config.getIntakeDeployKd());
    deployStowedPos =
        new LoggedTunableNumber(
            "Tuning/Intake/IntakeDeploy/StowedPosition", config.getIntakeDeployStowedPosition());
    deployExtendedPos =
        new LoggedTunableNumber(
            "Tuning/Intake/IntakeDeploy/ExtendedPosition",
            config.getIntakeDeployExtendedPosition());
    deployRetractedPos =
        new LoggedTunableNumber(
            "Tuning/Intake/IntakeDeploy/RetractedPosition",
            config.getIntakeDeployRetractedPosition());
    deployTolerance =
        new LoggedTunableNumber(
            "Tuning/Intake/IntakeDeploy/Tolerance", config.getIntakeDeployTolerance());
    holdTolerance =
        new LoggedTunableNumber(
            "Tuning/Intake/IntakeDeploy/HoldTolerance", config.getIntakeDeployHoldTolerance());
    rollerKP =
        new LoggedTunableNumber("Tuning/Intake/IntakeRollers/kP", config.getIntakeRollerKp());
    rollerKI =
        new LoggedTunableNumber("Tuning/Intake/IntakeRollers/kI", config.getIntakeRollerKi());
    rollerKD =
        new LoggedTunableNumber("Tuning/Intake/IntakeRollers/kD", config.getIntakeRollerKd());
    rollerKFF =
        new LoggedTunableNumber("Tuning/Intake/IntakeRollers/kFF", config.getIntakeRollerKff());
    deployKS = new LoggedTunableNumber("Tuning/Intake/IntakeDeploy/kS", config.getIntakeDeployKs());
    deployKV = new LoggedTunableNumber("Tuning/Intake/IntakeDeploy/kV", config.getIntakeDeployKv());
    // Deploy motion profile
    deployMaxVelocity =
        new LoggedTunableNumber(
            "Tuning/Intake/Deploy/MaxVelocity", config.getIntakeDeployMaxVelocity());
    deployMaxAcceleration =
        new LoggedTunableNumber(
            "Tuning/Intake/Deploy/MaxAcceleration", config.getIntakeDeployMaxAcceleration());
    deployOutputLimit =
        new LoggedTunableNumber(
            "Tuning/Intake/Deploy/OutputLimit", config.getIntakeDeployOutputLimit());
    deployBrakeTime =
        new LoggedTunableNumber(
            "Tuning/Intake/Deploy/BrakeTimeSec", config.getIntakeDeployBrakeTimeSec());
    // Retract motion profile
    retractMaxVelocity =
        new LoggedTunableNumber(
            "Tuning/Intake/Retract/MaxVelocity", config.getIntakeRetractMaxVelocity());
    retractMaxAcceleration =
        new LoggedTunableNumber(
            "Tuning/Intake/Retract/MaxAcceleration", config.getIntakeRetractMaxAcceleration());
    retractOutputLimit =
        new LoggedTunableNumber(
            "Tuning/Intake/Retract/OutputLimit", config.getIntakeRetractOutputLimit());
    // Agitation motion profile
    agitationMaxVelocity =
        new LoggedTunableNumber(
            "Tuning/Intake/Agitation/MaxVelocity", config.getIntakeAgitationMaxVelocity());
    agitationMaxAcceleration =
        new LoggedTunableNumber(
            "Tuning/Intake/Agitation/MaxAcceleration", config.getIntakeAgitationMaxAcceleration());
    agitationRetractOutputLimit =
        new LoggedTunableNumber(
            "Tuning/Intake/Agitation/RetractOutputLimit",
            config.getIntakeAgitationRetractOutputLimit());
    agitationRetractTarget =
        new LoggedTunableNumber(
            "Tuning/Intake/Agitation/RetractTarget", config.getIntakeAgitationRetractTarget());
    agitationTimeoutSec =
        new LoggedTunableNumber(
            "Tuning/Intake/Agitation/TimeoutSec", config.getIntakeAgitationTimeoutSec());
    agitationCoastTimeSec =
        new LoggedTunableNumber(
            "Tuning/Intake/Agitation/CoastTimeSec", config.getIntakeAgitationCoastTimeSec());
    agitationFallTime =
        new LoggedTunableNumber(
            "Tuning/Intake/Agitation/FallTimeSec", config.getIntakeAgitationFallTimeSec());
    agitationSpeedThreshold =
        new LoggedTunableNumber(
            "Tuning/Intake/Agitation/SpeedThresholdMps",
            config.getIntakeAgitationSpeedThresholdMps());
    agitationStationaryDwellSec =
        new LoggedTunableNumber(
            "Tuning/Intake/Agitation/StationaryDwellSec",
            config.getIntakeAgitationStationaryDwellSec());
    SmartDashboard.putBoolean("Intake/AgitationEnabled", false);
    // Shared
    rollerMinDeployPosition =
        new LoggedTunableNumber(
            "Tuning/Intake/IntakeDeploy/RollerMinDeployPosition",
            config.getIntakeRollerMinDeployPosition());
  }

  // Deploy positions (from config, used for soft limit init)
  private final double deployStowedPosition;
  private final double deployRetractedPosition;
  private final double deployExtendedPosition;

  // Tracks whether we've commanded deploy (true) or retract (false) — used for roller RPM selection
  private boolean deployCommanded = false;
  private boolean movingFirstCycle =
      false; // Skip deployAtTarget check on first cycle (stale inputs)

  // Deploy state machine
  private enum DeployState {
    RETRACTED,
    DEPLOYING,
    DEPLOY_SETTLING,
    DEPLOYED,
    RETRACTING,
    AGITATING,
    AGITATE_SETTLING
  }

  /** Roller operating state. */
  public enum RollerState {
    IDLE,
    INTAKING,
    EJECTING,
    SAFETY_LOCKED
  }

  private DeployState deployState = DeployState.RETRACTED;
  private final Timer brakeTimer = new Timer();
  // ========== MECHANISM 2D VISUALIZATION ==========
  // Canvas dimensions (meters) — side view of robot
  private static final double MECH_CANVAS_WIDTH = 1.0;
  private static final double MECH_CANVAS_HEIGHT = 0.6;
  // Pivot at right edge of robot body (front bumper)
  private static final double MECH_PIVOT_X = 0.50;
  private static final double MECH_PIVOT_Z = 0.20;
  // Visual arm length (meters) — extends past bumper when horizontal
  private static final double MECH_ARM_LENGTH = 0.30;
  private static final double MECH_ARM_LINE_WIDTH = 8.0;
  private static final double MECH_ROLLER_LENGTH = 0.06;
  private static final double MECH_ROLLER_LINE_WIDTH = 12.0;
  // Robot body reference line (static, extends leftward from pivot)
  private static final double MECH_BODY_LENGTH = 0.40;
  private static final double MECH_BODY_LINE_WIDTH = 6.0;

  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismLigament2d armLigament;
  private final LoggedMechanismLigament2d rollerLigament;

  // Color palette
  private static final Color8Bit COLOR_RETRACTED = new Color8Bit(Color.kGray);
  private static final Color8Bit COLOR_DEPLOYING = new Color8Bit(Color.kOrange);
  private static final Color8Bit COLOR_DEPLOYED = new Color8Bit(Color.kGreen);
  private static final Color8Bit COLOR_RETRACTING = new Color8Bit(Color.kOrange);
  private static final Color8Bit COLOR_AGITATING = new Color8Bit(Color.kYellow);
  private static final Color8Bit COLOR_SETTLING = new Color8Bit(Color.kCyan);
  private static final Color8Bit COLOR_ROLLER_INTAKE = new Color8Bit(Color.kLimeGreen);
  private static final Color8Bit COLOR_ROLLER_EJECT = new Color8Bit(Color.kRed);
  private static final Color8Bit COLOR_ROLLER_IDLE = new Color8Bit(Color.kDarkGray);
  private static final Color8Bit COLOR_ROLLER_LOCKED = new Color8Bit(Color.kDarkRed);
  private static final Color8Bit COLOR_BODY = new Color8Bit(Color.kDimGray);

  // Operational constants (not robot-specific)
  public static final double ROLLER_INTAKE_SPEED = 0.8;
  public static final double ROLLER_EJECT_SPEED = -0.6;
  public static final double ROLLER_HOLD_SPEED = 0.1;
  public static final double ROLLER_INTAKE_RPM_RETRACTED = 0.0;
  public static final double ROLLER_INTAKE_RPM_DEPLOYED = 2000.0;
  public static final double ROLLER_EJECT_RPM = -1000.0;

  // Pending roller velocity — set when deploy is commanded, applied once position threshold is met
  private boolean rollersPending = false;
  private double pendingRollerRPM = 0.0;
  private boolean rollersActivelyCommanded = false;
  private double activeRollerRPM = 0.0;

  // Velocity control toggle (default: velocity control on)
  private boolean useVelocityControl = true;

  // Optional: supplier for robot velocity (for velocity-based roller speed)
  private DoubleSupplier robotVelocitySupplier = () -> 0.0;

  /**
   * Creates a new Intake subsystem.
   *
   * @param io The IO implementation to use (real hardware or simulation)
   */
  public Intake(IntakeIO io) {
    this.io = io;

    RobotConfig config = Constants.getRobotConfig();
    deployStowedPosition = config.getIntakeDeployStowedPosition();
    deployRetractedPosition = config.getIntakeDeployRetractedPosition();
    deployExtendedPosition = config.getIntakeDeployExtendedPosition();

    // Command stowed position at startup so SparkMax has an active hold target
    applyRetractMotionConfig();
    io.setDeployBrakeMode(true);
    io.setDeployPosition(deployStowedPosition);

    // Initialize Mechanism2d side-view visualization
    mechanism = new LoggedMechanism2d(MECH_CANVAS_WIDTH, MECH_CANVAS_HEIGHT);

    // Robot body reference line — horizontal bar extending backward from pivot
    LoggedMechanismRoot2d bodyRoot = mechanism.getRoot("Body", MECH_PIVOT_X, MECH_PIVOT_Z);
    bodyRoot.append(
        new LoggedMechanismLigament2d(
            "RobotBody", MECH_BODY_LENGTH, 180, MECH_BODY_LINE_WIDTH, COLOR_BODY));

    // Intake arm — rotates from pivot; 90° = straight up (retracted), tilts forward when deployed
    LoggedMechanismRoot2d pivotRoot = mechanism.getRoot("IntakePivot", MECH_PIVOT_X, MECH_PIVOT_Z);
    armLigament =
        pivotRoot.append(
            new LoggedMechanismLigament2d(
                "IntakeArm", MECH_ARM_LENGTH, 90, MECH_ARM_LINE_WIDTH, COLOR_RETRACTED));

    // Roller indicator at the tip of the arm
    rollerLigament =
        armLigament.append(
            new LoggedMechanismLigament2d(
                "Roller", MECH_ROLLER_LENGTH, 0, MECH_ROLLER_LINE_WIDTH, COLOR_ROLLER_IDLE));
  }

  /**
   * Sets a supplier for robot velocity to adjust roller speed.
   *
   * @param velocitySupplier Supplier that returns robot linear velocity in m/s
   */
  public void setRobotVelocitySupplier(DoubleSupplier velocitySupplier) {
    this.robotVelocitySupplier = velocitySupplier;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Push tunable changes to IO
    if (LoggedTunableNumber.hasChanged(deployKP, deployKI, deployKD)) {
      io.configureDeployPID(deployKP.get(), deployKI.get(), deployKD.get());
    }
    if (LoggedTunableNumber.hasChanged(deployKS, deployKV)) {
      io.configureDeployFeedforward(deployKS.get(), deployKV.get());
    }
    if (LoggedTunableNumber.hasChanged(rollerKP, rollerKI, rollerKD, rollerKFF)) {
      io.configureRollerPID(rollerKP.get(), rollerKI.get(), rollerKD.get(), rollerKFF.get());
    }
    if (LoggedTunableNumber.hasChanged(deployOutputLimit, retractOutputLimit)) {
      io.configureDeployOutputRange(
          -Math.abs(retractOutputLimit.get()), Math.abs(deployOutputLimit.get()));
    }

    // Deploy state machine
    switch (deployState) {
      case DEPLOYING:
        // Skip first cycle — inputs.deployTargetPosition is stale from before the command
        if (movingFirstCycle) {
          movingFirstCycle = false;
          break;
        }
        if (deployAtTarget()) {
          // At extended position — brake to settle, then coast
          io.disableDeploy();
          io.setDeployBrakeMode(true);
          brakeTimer.restart();
          deployState = DeployState.DEPLOY_SETTLING;
        }
        break;
      case RETRACTING:
        // Skip first cycle — inputs.deployTargetPosition is stale from before the command
        if (movingFirstCycle) {
          movingFirstCycle = false;
          break;
        }
        if (retractAtTarget()) {
          // At retracted position — MAXMotion continues holding, brake mode already set
          deployState = DeployState.RETRACTED;
        }
        break;
      case DEPLOY_SETTLING:
        // At deployed position: after brake time elapses, switch to coast
        if (brakeTimer.hasElapsed(deployBrakeTime.get())) {
          brakeTimer.stop();
          io.disableDeploy();
          io.setDeployBrakeMode(false);
          deployState = DeployState.DEPLOYED;
        }
        break;
      case RETRACTED:
        // Brake mode + MAXMotion maintains position via onboard 1kHz PID — no action needed
        break;
      case AGITATING:
        // Agitate command owns motor control — no action needed from state machine
        break;
      case AGITATE_SETTLING:
        // Post-agitate: brake briefly, then return to deployed position
        if (brakeTimer.hasElapsed(agitationFallTime.get())) {
          brakeTimer.stop();
          deploy();
        }
        break;
      case DEPLOYED:
      default:
        break;
    }

    // Activate pending rollers once deploy reaches the activation position
    if (rollersPending && inputs.deployPositionRotations >= rollerMinDeployPosition.get()) {
      io.setRollerVelocity(pendingRollerRPM);
      rollersPending = false;
    }

    // Safety interlock: force rollers off when deploy is too close to stowed.
    // If rollers were actively running (rollersActivelyCommanded == true), re-pend them
    // so they automatically restart when the arm recovers past rollerMinDeployPosition.
    // This relies on callers (runIntake, setRollerVelocityWhenDeployed) setting
    // rollersActivelyCommanded = true — see runIntake() fix for details.
    boolean rollersSafetyLocked = inputs.deployPositionRotations < rollerMinDeployPosition.get();
    if (rollersSafetyLocked) {
      if (rollersActivelyCommanded && !rollersPending) {
        pendingRollerRPM = activeRollerRPM;
        rollersPending = true;
      }
      io.stopRollerMotor();
    }

    // ---- Update Mechanism2d visualization ----
    // Map deploy position to visual angle:
    //   position 0 (stowed)    → 90° (straight up)
    //   position extended      → 0°  (horizontal, pointing forward past bumper)
    double deployFraction = inputs.deployPositionRotations / deployExtendedPos.get();
    double armAngleDeg = 90.0 * (1.0 - Math.min(deployFraction, 1.0));
    armLigament.setAngle(armAngleDeg);

    // Arm color based on deploy state
    switch (deployState) {
      case RETRACTED -> armLigament.setColor(COLOR_RETRACTED);
      case DEPLOYING -> armLigament.setColor(COLOR_DEPLOYING);
      case DEPLOY_SETTLING -> armLigament.setColor(COLOR_SETTLING);
      case DEPLOYED -> armLigament.setColor(COLOR_DEPLOYED);
      case RETRACTING -> armLigament.setColor(COLOR_RETRACTING);
      case AGITATING -> armLigament.setColor(COLOR_AGITATING);
      case AGITATE_SETTLING -> armLigament.setColor(COLOR_SETTLING);
    }

    // Roller color based on roller state
    if (rollersSafetyLocked) {
      rollerLigament.setColor(COLOR_ROLLER_LOCKED);
    } else if (inputs.rollerVelocityRPM > 50) {
      rollerLigament.setColor(COLOR_ROLLER_INTAKE);
    } else if (inputs.rollerVelocityRPM < -50) {
      rollerLigament.setColor(COLOR_ROLLER_EJECT);
    } else {
      rollerLigament.setColor(COLOR_ROLLER_IDLE);
    }

    Logger.recordOutput("Visualizations/Intake2d", mechanism);

    // Log state machines
    Logger.recordOutput("Subsystems/IntakeDeployState", deployState.name());

    RollerState rollerState;
    if (rollersSafetyLocked) {
      rollerState = RollerState.SAFETY_LOCKED;
    } else if (inputs.rollerVelocityRPM > 50) {
      rollerState = RollerState.INTAKING;
    } else if (inputs.rollerVelocityRPM < -50) {
      rollerState = RollerState.EJECTING;
    } else {
      rollerState = RollerState.IDLE;
    }
    Logger.recordOutput("Subsystems/IntakeRollerState", rollerState.name());
  }

  // ========== DEPLOY CONTROL ==========

  /** Apply deploy (extend) MAXMotion profile to the motor. */
  private void applyDeployMotionConfig() {
    io.configureDeployMaxMotion(
        deployMaxVelocity.get(), deployMaxAcceleration.get(), deployTolerance.get());
  }

  /** Apply retract/stow MAXMotion profile to the motor. */
  private void applyRetractMotionConfig() {
    io.configureDeployMaxMotion(
        retractMaxVelocity.get(), retractMaxAcceleration.get(), deployTolerance.get());
  }

  /** Deploy the intake (extend). Stops motor first for clean retarget. */
  public void deploy() {
    // Already deployed or deploying — don't restart the sequence
    if (deployState == DeployState.DEPLOYING
        || deployState == DeployState.DEPLOY_SETTLING
        || deployState == DeployState.DEPLOYED) {
      return;
    }
    io.stopDeploy(); // Cancel any in-progress motion before commanding new target
    io.setDeployBrakeMode(true); // Brake mode while PID is driving
    applyDeployMotionConfig();
    deployCommanded = true;
    movingFirstCycle = true;
    deployState = DeployState.DEPLOYING;
    io.setDeployPosition(deployExtendedPos.get());
  }

  /** Retract the intake. Stops motor first for clean retarget. */
  public void retract() {
    io.stopDeploy(); // Cancel any in-progress motion before commanding new target
    io.setDeployBrakeMode(
        true); // Brake mode for retract — provides backstop during and after motion
    applyRetractMotionConfig();
    deployCommanded = false;
    rollersPending = false;
    rollersActivelyCommanded = false;
    movingFirstCycle = true;
    deployState = DeployState.RETRACTING;
    io.setDeployPosition(deployRetractedPos.get());
  }

  /**
   * Stow the intake (fully retracted past normal retract position). Stops motor first for clean
   * retarget.
   */
  public void stow() {
    io.stopDeploy(); // Cancel any in-progress motion before commanding new target
    io.setDeployBrakeMode(true); // Brake mode for stow — provides backstop during and after motion
    applyRetractMotionConfig();
    deployCommanded = false;
    movingFirstCycle = true;
    deployState = DeployState.RETRACTING;
    io.setDeployPosition(deployStowedPos.get());
  }

  /** Emergency stop the deploy motor. Switches to brake mode to hold position. */
  public void stopDeploy() {
    io.stopDeploy();
    io.setDeployBrakeMode(true);
    deployCommanded = false;
    rollersPending = false;
    rollersActivelyCommanded = false;
    deployState = DeployState.RETRACTED;
    brakeTimer.stop();
  }

  /** Command that immediately stops the deploy motor (bind to a button for safety). */
  public Command stopDeployCommand() {
    return runOnce(this::stopDeploy).withName("Intake: Stop Deploy");
  }

  /** Command that stops everything — deploy motor + rollers. */
  public Command stopAllCommand() {
    return runOnce(
            () -> {
              stopDeploy();
              stopRollers();
            })
        .withName("Intake: Stop All");
  }

  /**
   * Set the deploy position directly.
   *
   * @param positionRotations Target position in rotations
   */
  public void setDeployPosition(double positionRotations) {
    io.setDeployPosition(positionRotations);
  }

  /** Check if the intake is at or past the deployed position (accepts overshoot from gravity). */
  public boolean isDeployed() {
    return inputs.deployPositionRotations >= deployExtendedPos.get() - deployTolerance.get();
  }

  /** Check if the intake is fully retracted. */
  public boolean isRetracted() {
    return Math.abs(inputs.deployPositionRotations - deployRetractedPos.get())
        <= deployTolerance.get();
  }

  /** Check if the intake is fully stowed. */
  public boolean isStowed() {
    return Math.abs(inputs.deployPositionRotations - deployStowedPos.get())
        <= deployTolerance.get();
  }

  /** Check if the deploy mechanism is at target. */
  public boolean deployAtTarget() {
    return inputs.deployPositionRotations >= inputs.deployTargetPosition;
  }

  /** Check if the deploy mechanism is at target. */
  public boolean retractAtTarget() {
    return Math.abs(inputs.deployPositionRotations - inputs.deployTargetPosition)
        <= deployTolerance.get();
  }

  /** Get the current deploy position. */
  public double getDeployPosition() {
    return inputs.deployPositionRotations;
  }

  // ========== ROLLER CONTROL ==========

  /** Returns true if rollers are blocked by the deploy safety interlock. */
  private boolean isRollerSafetyLocked() {
    return inputs.deployPositionRotations < rollerMinDeployPosition.get();
  }

  /** Returns whether velocity control is active. */
  public boolean isVelocityControlEnabled() {
    return useVelocityControl;
  }

  /** Toggle between velocity control and open-loop duty cycle control. */
  public void toggleVelocityControl() {
    useVelocityControl = !useVelocityControl;
  }

  /** Set whether to use velocity control (true) or open-loop (false). */
  public void setVelocityControlEnabled(boolean enabled) {
    useVelocityControl = enabled;
  }

  /** Command that toggles velocity control mode. */
  public Command toggleVelocityControlCommand() {
    return runOnce(this::toggleVelocityControl).withName("Intake: Toggle Velocity Control");
  }

  /**
   * Run rollers to intake game pieces. RPM varies based on deploy state.
   *
   * <p>FIX: Previously, this method did not set rollersActivelyCommanded or activeRollerRPM. This
   * caused rollers to permanently stop during auto if the safety interlock triggered (arm dipping
   * below rollerMinDeployPosition). The safety re-pend logic in periodic() checks
   * rollersActivelyCommanded to decide whether to re-pend stopped rollers — without it being set,
   * rollers would never restart after a momentary safety lock. Now we set both fields up front so
   * the safety interlock can properly re-pend and restart the rollers.
   */
  public void runIntake() {
    double rpm = deployCommanded ? ROLLER_INTAKE_RPM_DEPLOYED : ROLLER_INTAKE_RPM_RETRACTED;

    // Mark rollers as actively commanded so the safety interlock in periodic() can
    // re-pend them if the arm temporarily dips below the safe position threshold.
    // Without this, a momentary safety lock during auto would kill rollers permanently.
    activeRollerRPM = rpm;
    rollersActivelyCommanded = true;

    if (isRollerSafetyLocked()) {
      // Arm is still too close to stowed — defer roller start until deploy reaches safe position
      if (useVelocityControl) {
        pendingRollerRPM = rpm;
        rollersPending = true;
      }
      return;
    }
    if (useVelocityControl) {
      io.setRollerVelocity(rpm);
    } else {
      io.setRollerDutyCycle(ROLLER_INTAKE_SPEED);
    }
  }

  /** Run rollers to eject game pieces. */
  public void runEject() {
    if (isRollerSafetyLocked()) return;
    if (useVelocityControl) {
      io.setRollerVelocity(ROLLER_EJECT_RPM);
    } else {
      io.setRollerDutyCycle(ROLLER_EJECT_SPEED);
    }
  }

  /** Stop the rollers. */
  public void stopRollers() {
    rollersPending = false;
    rollersActivelyCommanded = false;
    io.stopRollerMotor();
  }

  /**
   * Set roller speed directly as duty cycle (always open-loop).
   *
   * @param dutyCycle Duty cycle from -1 to 1
   */
  public void setRollerSpeed(double dutyCycle) {
    if (isRollerSafetyLocked()) return;
    io.setRollerDutyCycle(dutyCycle);
  }

  /**
   * Set roller velocity directly (closed-loop RPM).
   *
   * @param rpm Target roller velocity in RPM
   */
  public void setRollerVelocity(double rpm) {
    if (isRollerSafetyLocked()) return;
    io.setRollerVelocity(rpm);
  }

  /**
   * Request roller velocity that will activate once the deploy arm reaches the activation position.
   * If already past the threshold, rollers start immediately.
   *
   * @param rpm Target roller velocity in RPM
   */
  public void setRollerVelocityWhenDeployed(double rpm) {
    activeRollerRPM = rpm;
    rollersActivelyCommanded = true;
    if (inputs.deployPositionRotations >= rollerMinDeployPosition.get()) {
      io.setRollerVelocity(rpm);
      rollersPending = false;
    } else {
      pendingRollerRPM = rpm;
      rollersPending = true;
    }
  }

  /**
   * Run rollers with speed adjusted for robot velocity. Faster robot = faster rollers to maintain
   * grip on game pieces.
   *
   * @param baseSpeed Base roller speed (duty cycle)
   * @param velocityFactor How much robot velocity affects roller speed (duty cycle per m/s)
   */
  public void runIntakeWithVelocityCompensation(double baseSpeed, double velocityFactor) {
    if (isRollerSafetyLocked()) return;
    double robotVelocity = Math.abs(robotVelocitySupplier.getAsDouble());
    double compensatedSpeed = baseSpeed + (robotVelocity * velocityFactor);
    compensatedSpeed = Math.min(1.0, Math.max(-1.0, compensatedSpeed)); // Clamp to valid range
    io.setRollerDutyCycle(compensatedSpeed);
  }

  /** Get the current roller velocity in RPM. */
  public double getRollerVelocityRPM() {
    return inputs.rollerVelocityRPM;
  }

  /** Get the roller current draw (useful for game piece detection). */
  public double getRollerCurrentAmps() {
    return inputs.rollerCurrentAmps;
  }

  /** Check if the deploy is settled (deployed or retracted, no motion in progress). */
  public boolean isDeploySettled() {
    return deployState == DeployState.DEPLOYED || deployState == DeployState.RETRACTED;
  }

  /**
   * Command that agitates the intake to shake game pieces toward the spindexer. Uses a burst of
   * duty cycle power to jerk the arm up (bypassing MAXMotion output limits), then releases to coast
   * so gravity pulls it back down. Repeats this cycle. Pauses when robot speed exceeds threshold.
   * Rollers spin during agitation.
   *
   * @param rollerRPM Supplier for roller velocity in RPM
   * @return Command that agitates until cancelled
   */
  /** Apply agitation-specific output range and acceleration to the deploy motor. */
  private void applyAgitationConfig() {
    io.configureDeployOutputRange(
        -Math.abs(agitationRetractOutputLimit.get()), Math.abs(deployOutputLimit.get()));
    io.configureDeployMaxMotion(
        agitationMaxVelocity.get(), agitationMaxAcceleration.get(), deployTolerance.get());
  }

  /** Restore normal output range after agitation. */
  private void restoreNormalDeployConfig() {
    io.configureDeployOutputRange(
        -Math.abs(retractOutputLimit.get()), Math.abs(deployOutputLimit.get()));
  }

  /**
   * @param rollerRPM Supplier for roller velocity in RPM
   * @param intakeActive Supplier that returns true when the deploy button is held (actively
   *     intaking). While true, agitation is suppressed — the arm stays put and only rollers run.
   * @return Command that agitates until cancelled
   */
  public Command agitateCommand(DoubleSupplier rollerRPM, BooleanSupplier intakeActive) {
    Timer agitationTimer = new Timer();
    Timer stationaryTimer = new Timer();
    // Remember pre-agitation state so we can restore it when done
    boolean[] wasDeployed = {false};
    // NOTE: This command intentionally does NOT require the intake subsystem
    // (uses Commands.runOnce/run instead of this.runOnce/run) so it can coexist
    // with Button 4's deploy+roller command. When intakeActive is true, this
    // command idles in Phase 1 without touching any motors, letting Button 4
    // retain full control. When intakeActive becomes false, this command takes
    // over the deploy motor for agitation.
    return Commands.runOnce(
            () -> {
              wasDeployed[0] = deployCommanded;
              stationaryTimer.restart();
            })
        .andThen(
            Commands.waitUntil(
                    () ->
                        inputs.deployPositionRotations
                            >= deployExtendedPos.get() - deployTolerance.get())
                .andThen(
                    Commands.sequence(
                            // Phase 1: Wait until intake button is released AND robot has been
                            // stationary for the dwell period. Any movement or active intaking
                            // resets the timer.
                            Commands.runOnce(stationaryTimer::restart),
                            Commands.run(
                                    () -> {
                                      if (intakeActive.getAsBoolean()
                                          || Math.abs(robotVelocitySupplier.getAsDouble())
                                              >= agitationSpeedThreshold.get()) {
                                        stationaryTimer.restart();
                                      }
                                    })
                                .until(
                                    () ->
                                        stationaryTimer.hasElapsed(
                                                agitationStationaryDwellSec.get())
                                            && SmartDashboard.getBoolean(
                                                "Intake/AgitationEnabled", false)),
                            // Phase 2: Agitation cycles — run while stationary and not intaking
                            Commands.sequence(
                                    // UP phase: MAXMotion position control retracts against gravity
                                    Commands.runOnce(
                                        () -> {
                                          applyAgitationConfig();
                                          io.setDeployBrakeMode(false);
                                          io.setDeployPosition(agitationRetractTarget.get());
                                          deployState = DeployState.AGITATING;
                                          setRollerVelocityWhenDeployed(rollerRPM.getAsDouble());
                                          agitationTimer.restart();
                                        }),
                                    Commands.waitUntil(
                                        () ->
                                            Math.abs(
                                                        inputs.deployPositionRotations
                                                            - agitationRetractTarget.get())
                                                    <= deployTolerance.get()
                                                || agitationTimer.hasElapsed(
                                                    agitationTimeoutSec.get())),
                                    // DOWN phase — coast: motor off, coast mode, gravity pulls arm
                                    Commands.runOnce(
                                        () -> {
                                          restoreNormalDeployConfig();
                                          io.disableDeploy();
                                          io.setDeployBrakeMode(false);
                                        }),
                                    Commands.waitSeconds(agitationCoastTimeSec.get()),
                                    // DOWN phase — brake: back-EMF braking decelerates the arm
                                    Commands.runOnce(() -> io.setDeployBrakeMode(true)),
                                    Commands.waitSeconds(
                                        Math.max(
                                            0,
                                            agitationFallTime.get() - agitationCoastTimeSec.get())))
                                .repeatedly()
                                .onlyWhile(
                                    () ->
                                        !intakeActive.getAsBoolean()
                                            && Math.abs(robotVelocitySupplier.getAsDouble())
                                                < agitationSpeedThreshold.get())
                                .finallyDo(
                                    () -> {
                                      // Agitation interrupted — send arm back to deployed position
                                      // so it's not left dangling mid-cycle
                                      restoreNormalDeployConfig();
                                      deploy();
                                    }))
                        // Repeat: if robot moves or intake button pressed mid-agitation,
                        // go back to waiting for release + full stationary dwell
                        .repeatedly()))
        .finallyDo(
            () -> {
              restoreNormalDeployConfig();
              io.stopDeploy(); // Anchor target to current pos before configure() calls
              io.setDeployBrakeMode(true);
              if (wasDeployed[0]) {
                // Was deployed before agitation — resume rollers so the next path can intake
                runIntake();
                brakeTimer.restart();
                deployState = DeployState.AGITATE_SETTLING;
              } else {
                // Was retracted before agitation — go straight to retract hold
                stopRollers();
                applyRetractMotionConfig();
                io.setDeployPosition(deployStowedPosition);
                deployState = DeployState.RETRACTED;
              }
            })
        .withName("Intake: Agitate");
  }

  // ========== Commands ==========

  /**
   * Command to deploy the intake and run rollers at the supplied RPM. On cancel, stops rollers and
   * retracts.
   *
   * @param rollerRPM Supplier for roller velocity in RPM (read each cycle for live tuning)
   * @return Command that deploys + runs until interrupted
   */
  public Command deployAndRunCommand(DoubleSupplier rollerRPM) {
    return runOnce(this::deploy)
        .andThen(run(() -> setRollerVelocity(rollerRPM.getAsDouble())))
        .finallyDo(
            () -> {
              stopRollers();
              retract();
            })
        .withName("Intake: Deploy & Run");
  }

  /**
   * Command that runs intake rollers during SmartLaunch. Does NOT deploy or retract the intake —
   * rollers only spin when the arm is already deployed (via the existing safety interlock in {@link
   * #setRollerVelocityWhenDeployed}). Does NOT require the intake subsystem so it coexists with
   * Button 4's deploy+roller command.
   *
   * @param rollerRPM Supplier for roller velocity in RPM
   * @param intakeActive Supplier that returns true when Button 4 is held (yields roller control)
   * @return Command that runs rollers until cancelled
   */
  public Command smartLaunchRollerCommand(DoubleSupplier rollerRPM, BooleanSupplier intakeActive) {
    return Commands.run(
            () -> {
              if (!intakeActive.getAsBoolean()) {
                setRollerVelocityWhenDeployed(rollerRPM.getAsDouble());
              }
            })
        .finallyDo(
            () -> {
              if (!intakeActive.getAsBoolean()) {
                stopRollers();
              }
            })
        .withName("Intake: SmartLaunch Rollers");
  }

  // ========== GENERAL CONTROL ==========

  /** Stop all motors. */
  public void stop() {
    io.stop();
  }
}
