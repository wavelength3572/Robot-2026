package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/**
 * Climber subsystem — three positions, two buttons.
 *
 * <p>Hardware: single NEO motor through a 25:1 gearbox driving a 0.75" winch drum. Brake mode holds
 * position when the motor stops.
 *
 * <p>Positions:
 *
 * <ul>
 *   <li>Stowed (0) — fully retracted, put away
 *   <li>Extended (extendPosition) — arm up, lined up with the pole
 *   <li>Climbed (climbPosition) — partially retracted, robot off the ground
 * </ul>
 *
 * <p>Operator buttons:
 *
 * <ul>
 *   <li>B9 (extend): STOWED→EXTENDED, CLIMBED→EXTENDED, EXTENDED→STOWED
 *   <li>B2-1 (climb): EXTENDED→CLIMBED
 * </ul>
 */
public class Climber extends SubsystemBase {

  public enum ClimberState {
    STOWED, // Position 0 — put away
    EXTENDING, // Moving to extended position
    EXTENDED, // Arm up, lined up with pole
    CLIMBING, // Moving to climb position
    CLIMBED, // Off the ground, brake holds
    STOWING // Moving back to position 0
  }

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private ClimberState state = ClimberState.STOWED;
  private double targetPositionRotations = 0.0;

  private static final LoggedTunableNumber extendPosition =
      new LoggedTunableNumber(
          "Climber/extendPosition", Constants.getRobotConfig().getClimberExtendPosition());
  private static final LoggedTunableNumber climbPosition =
      new LoggedTunableNumber(
          "Climber/climbPosition", Constants.getRobotConfig().getClimberClimbPosition());
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Climber/kP", Constants.getRobotConfig().getClimberKp());
  private static final LoggedTunableNumber positionTolerance =
      new LoggedTunableNumber("Climber/positionTolerance", 2.0);
  private static final LoggedTunableNumber extendTimeoutSec =
      new LoggedTunableNumber("Climber/extendTimeoutSec", 5.0);
  private static final LoggedTunableNumber climbTimeoutSec =
      new LoggedTunableNumber("Climber/climbTimeoutSec", 5.0);

  private boolean lastClimbSucceeded = false;

  private final Alert climberNotStowedAlert =
      new Alert("Climber is not stowed — hold B9 for 2s to stow", AlertType.kWarning);

  // Pitch/roll suppliers for climbing observability (wired by RobotContainer)
  private DoubleSupplier pitchSupplier = () -> 0.0;
  private DoubleSupplier rollSupplier = () -> 0.0;

  // ========== MECHANISM 2D VISUALIZATION ==========
  // Canvas: side view, tall enough to show full extension + loop
  private static final double MECH_CANVAS_WIDTH = 1.0; // meters
  private static final double MECH_CANVAS_HEIGHT = 1.0; // meters
  // Climber base at back edge of robot frame
  private static final double MECH_BASE_X = 0.10;
  private static final double MECH_BASE_Z = 0.15; // frame rail height off floor
  // Robot body: extends forward (rightward) from climber base
  private static final double MECH_BODY_LENGTH = 0.787; // bumper-to-bumper
  private static final double MECH_BODY_LINE_WIDTH = 6.0;
  // Arm: extends vertically from base. Max length sized so tip = low rung height (27")
  private static final double MECH_ARM_MIN_LENGTH = 0.05; // stowed stub
  private static final double MECH_ARM_MAX_LENGTH =
      Units.inchesToMeters(27.0) - MECH_BASE_Z; // tip reaches low rung when extended
  private static final double MECH_ARM_LINE_WIDTH = 6.0;
  // Closed rectangular loop at tip — pole goes through front-to-back, so from
  // side view the rectangle is tall (vertical) and narrow (horizontal)
  private static final double MECH_HOOK_HEIGHT = 0.10; // tall dimension (vertical)
  private static final double MECH_HOOK_WIDTH = 0.06; // narrow dimension (horizontal, side-to-side)
  private static final double MECH_HOOK_LINE_WIDTH = 4.0;
  // Colors
  private static final Color8Bit COLOR_STOWED = new Color8Bit(Color.kGray);
  private static final Color8Bit COLOR_MOVING = new Color8Bit(Color.kOrange);
  private static final Color8Bit COLOR_EXTENDED = new Color8Bit(Color.kGreen);
  private static final Color8Bit COLOR_CLIMBED = new Color8Bit(Color.kDodgerBlue);
  private static final Color8Bit COLOR_BODY = new Color8Bit(Color.kDimGray);
  private static final Color8Bit COLOR_LOOP = new Color8Bit(Color.kWhite);

  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismLigament2d armLigament;

  public Climber(ClimberIO io) {
    this.io = io;

    // Initialize Mechanism2d side-view visualization
    mechanism = new LoggedMechanism2d(MECH_CANVAS_WIDTH, MECH_CANVAS_HEIGHT);

    // Robot body — horizontal reference line extending forward from back edge
    LoggedMechanismRoot2d bodyRoot = mechanism.getRoot("Body", MECH_BASE_X, MECH_BASE_Z);
    bodyRoot.append(
        new LoggedMechanismLigament2d(
            "RobotBody", MECH_BODY_LENGTH, 0, MECH_BODY_LINE_WIDTH, COLOR_BODY));

    // Climber arm — vertical stick growing upward from back edge
    LoggedMechanismRoot2d armRoot = mechanism.getRoot("ClimberBase", MECH_BASE_X, MECH_BASE_Z);
    armLigament =
        armRoot.append(
            new LoggedMechanismLigament2d(
                "ClimberArm", MECH_ARM_MIN_LENGTH, 90, MECH_ARM_LINE_WIDTH, COLOR_STOWED));

    // Closed rectangular loop at tip of arm (side view).
    // Post connects to center of the bottom (long) side.
    // The pole sits inside the rectangle.
    //
    //      _________
    //     |         |
    //     |____|____|
    //          |
    //          |  <- post (centered on bottom)
    //
    // From top of arm: left half-width, up, right full-width, down, left half-width.

    // Bottom-left half: go left (backward)
    LoggedMechanismLigament2d hookBotLeft =
        armLigament.append(
            new LoggedMechanismLigament2d(
                "HookBotLeft", MECH_HOOK_WIDTH / 2, 90, MECH_HOOK_LINE_WIDTH, COLOR_LOOP));
    // Left side: go up
    LoggedMechanismLigament2d hookLeft =
        hookBotLeft.append(
            new LoggedMechanismLigament2d(
                "HookLeft", MECH_HOOK_HEIGHT, -90, MECH_HOOK_LINE_WIDTH, COLOR_LOOP));
    // Top: go right (forward, full width)
    LoggedMechanismLigament2d hookTop =
        hookLeft.append(
            new LoggedMechanismLigament2d(
                "HookTop", MECH_HOOK_WIDTH, -90, MECH_HOOK_LINE_WIDTH, COLOR_LOOP));
    // Right side: go down
    LoggedMechanismLigament2d hookRight =
        hookTop.append(
            new LoggedMechanismLigament2d(
                "HookRight", MECH_HOOK_HEIGHT, -90, MECH_HOOK_LINE_WIDTH, COLOR_LOOP));
    // Bottom-right half: go left (back toward post center)
    hookRight.append(
        new LoggedMechanismLigament2d(
            "HookBotRight", MECH_HOOK_WIDTH / 2, -90, MECH_HOOK_LINE_WIDTH, COLOR_LOOP));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    if (LoggedTunableNumber.hasChanged(kP)) {
      io.configurePID(kP.get());
    }

    switch (state) {
      case STOWED:
        break;

      case EXTENDING:
        if (atPosition(targetPositionRotations)) {
          io.stop();
          state = ClimberState.EXTENDED;
        }
        break;

      case EXTENDED:
        break;

      case CLIMBING:
        if (atPosition(targetPositionRotations)) {
          io.stop();
          state = ClimberState.CLIMBED;
          lastClimbSucceeded = true;
        }
        break;

      case CLIMBED:
        break;

      case STOWING:
        if (atPosition(targetPositionRotations)) {
          io.stop();
          state = ClimberState.STOWED;
        }
        break;
    }

    // Alert operator if climber is not stowed during teleop
    climberNotStowedAlert.set(DriverStation.isTeleopEnabled() && state != ClimberState.STOWED);

    Logger.recordOutput("Climber/State", state.name());
    Logger.recordOutput("Climber/TargetPosition", targetPositionRotations);
    Logger.recordOutput("Climber/LastClimbSucceeded", lastClimbSucceeded);
    Logger.recordOutput("Climber/PitchDeg", pitchSupplier.getAsDouble());
    Logger.recordOutput("Climber/RollDeg", rollSupplier.getAsDouble());

    // Update Mechanism2d visualization
    double fraction = Math.max(0.0, Math.min(inputs.positionRotations / extendPosition.get(), 1.0));
    double armLength = MECH_ARM_MIN_LENGTH + fraction * (MECH_ARM_MAX_LENGTH - MECH_ARM_MIN_LENGTH);
    armLigament.setLength(armLength);

    Color8Bit armColor =
        switch (state) {
          case STOWED -> COLOR_STOWED;
          case EXTENDING, CLIMBING, STOWING -> COLOR_MOVING;
          case EXTENDED -> COLOR_EXTENDED;
          case CLIMBED -> COLOR_CLIMBED;
        };
    armLigament.setColor(armColor);
    Logger.recordOutput("Visualizations/Climber2d", mechanism);

    // 3D component pose for AdvantageScope — climber moves vertically
    // Delta from stowed position using drum math, with tunable correction
    double gearRatio = Constants.getRobotConfig().getClimberGearRatio();
    double drumDiameterMeters =
        Units.inchesToMeters(Constants.getRobotConfig().getClimberDrumDiameterInches());
    double linearHeightMeters =
        (inputs.positionRotations / gearRatio) * Math.PI * drumDiameterMeters;
    Logger.recordOutput(
        "Visualizations/Climber", new Pose3d(0.0, 0.0, linearHeightMeters, new Rotation3d()));
  }

  // ===== Actions =====

  /** Extend action. Goes to extended position from STOWED or CLIMBED. No-op otherwise. */
  public void extend() {
    if (state == ClimberState.STOWED || state == ClimberState.CLIMBED) {
      targetPositionRotations = extendPosition.get();
      io.setPosition(targetPositionRotations);
      state = ClimberState.EXTENDING;
    }
  }

  /** Stow action. Returns to stowed position from EXTENDED only. No-op otherwise. */
  public void stow() {
    if (state == ClimberState.EXTENDED) {
      targetPositionRotations = 0.0;
      io.setPosition(targetPositionRotations);
      state = ClimberState.STOWING;
    }
  }

  /** Climb action. From EXTENDED, retracts to climb position. No-op otherwise. */
  public void climb() {
    lastClimbSucceeded = false;
    if (state == ClimberState.EXTENDED) {
      targetPositionRotations = climbPosition.get();
      io.setPosition(targetPositionRotations);
      state = ClimberState.CLIMBING;
    }
  }

  // ===== State queries =====

  public ClimberState getState() {
    return state;
  }

  public boolean isExtended() {
    return state == ClimberState.EXTENDED;
  }

  public boolean isClimbed() {
    return state == ClimberState.CLIMBED;
  }

  public boolean isStowed() {
    return state == ClimberState.STOWED;
  }

  public boolean didLastClimbSucceed() {
    return lastClimbSucceeded;
  }

  // ===== Observability suppliers =====

  public void setPitchSupplier(DoubleSupplier supplier) {
    this.pitchSupplier = supplier;
  }

  public void setRollSupplier(DoubleSupplier supplier) {
    this.rollSupplier = supplier;
  }

  // ===== Command factories (for auto — explicit, not toggles) =====

  /** Command: extend from stowed, finishes when extended or timeout. */
  public Command extendCommand() {
    return Commands.runOnce(this::extend, this)
        .andThen(Commands.waitUntil(this::isExtended).withTimeout(extendTimeoutSec.get()))
        .withName("ClimberExtend");
  }

  /** Command: climb from extended, finishes when climbed or timeout. Stops motor on timeout. */
  public Command climbCommand() {
    return Commands.runOnce(
            () -> {
              lastClimbSucceeded = false;
              climb();
            },
            this)
        .andThen(Commands.waitUntil(this::isClimbed).withTimeout(climbTimeoutSec.get()))
        .finallyDo(
            () -> {
              if (!isClimbed()) {
                io.stop();
              }
            })
        .withName("ClimberClimb");
  }

  // ===== Helpers =====

  private boolean atPosition(double targetRotations) {
    return Math.abs(inputs.positionRotations - targetRotations) < positionTolerance.get();
  }
}
