# Multi-Robot Configuration Guide

## Overview

This codebase supports two robots: **MainBot2026** and **MiniBot2026**. The robot configuration is selected at compile time using the `Constants.currentRobot` variable.

## Switching Between Robots

To switch which robot configuration is used, edit `src/main/java/frc/robot/Constants.java`:

```java
public static final RobotType currentRobot = RobotType.MAINBOT;  // or RobotType.MINIBOT
```

After changing this value:
1. Rebuild the project: `./gradlew build`
2. Deploy to the robot: `./gradlew deploy`

## Robot Configurations

### MainBot2026
- **Chassis**: 21.25" × 21.25" (0.54m × 0.54m)
- **Drive Motors**: REV NEO (4x)
- **Turn Motors**: REV NEO (4x)
- **Wheel Radius**: 1.97" (0.050m)
- **Drive Gear Ratio**: 6.12:1 (SDS MK4i L2)
- **Max Speed**: ~4.64 m/s
- **Mass**: 63.2 kg (robot + battery)
- **Subsystems**: Drive, Turret
- **CAN IDs**:
  - Pigeon: 19
  - Drive Motors: 5, 8, 3, 6 (FL, BL, FR, BR)
  - Turn Motors: 9, 4, 7, 2 (FL, BL, FR, BR)
  - CANCoders: 13, 12, 11, 10 (FL, BL, FR, BR)

### MiniBot2026
- **Chassis**: 14" × 14" (0.3556m × 0.3556m)
- **Drive Motors**: REV NEO Vortex (4x)
- **Turn Motors**: REV NEO 550 (4x)
- **Wheel Radius**: 2.0" (0.0508m)
- **Drive Gear Ratio**: 7.0556:1 (SDS MK4i L2)
- **Max Speed**: ~5.39 m/s
- **Mass**: 74.088 kg
- **Subsystems**: Drive (no turret)
- **CAN IDs**:
  - Pigeon: 19
  - Drive Motors: 11, 21, 31, 41 (FL, FR, BL, BR)
  - Turn Motors: 12, 22, 32, 42 (FL, FR, BL, BR)

## Implementation Details

### File Structure
- **`Constants.java`**: Robot type selection and configuration retrieval
- **`RobotConfig.java`**: Interface defining all robot-specific values
- **`MainBotConfig.java`**: MainBot implementation with Robot-2025 values
- **`MiniBotConfig.java`**: MiniBot implementation with MiniBot-2025 values
- **`DriveConstants.java`**: Unified interface that pulls from active robot config
- **`RobotContainer.java`**: Conditional subsystem instantiation

### How It Works

1. At startup, `Constants.getRobotConfig()` creates the appropriate config object based on `currentRobot`
2. `DriveConstants` retrieves all values from the active config
3. Subsystems use `DriveConstants` as before (no changes needed)
4. Turret subsystem is only instantiated when `currentRobot == MAINBOT`

### Adding New Robot-Specific Values

To add new robot-specific constants:

1. Add method to `RobotConfig` interface
2. Implement method in both `MainBotConfig` and `MiniBotConfig`
3. Access the value via `Constants.getRobotConfig().getYourNewMethod()`

### Logging

The robot type is automatically logged to AdvantageKit metadata as "RobotType". You can verify which robot is running by checking the logs.

## Calibration Notes

### MainBot
- Module zero rotations are calibrated (from Robot-2025)
- PID values are tuned for NEO drive motors

### MiniBot
- **Module zero rotations need calibration** (currently set to 0.0)
- PID values are set for NEO Vortex drive motors
- Tune these values when first testing on hardware

## Safety Checklist

Before deploying to either robot:

- [ ] Verify `Constants.currentRobot` matches the physical robot
- [ ] Check CAN IDs match your wiring
- [ ] Confirm max speed limits are appropriate
- [ ] Test with robot elevated/disabled first
- [ ] Verify module orientations are correct

## Testing in Simulation

The simulation works with both robot configurations:

```bash
# Test MainBot
# (Set Constants.currentRobot = RobotType.MAINBOT)
./gradlew simulateJava

# Test MiniBot
# (Set Constants.currentRobot = RobotType.MINIBOT)
./gradlew simulateJava
```

You should see different physical behaviors (size, speed, inertia) based on the selected robot.

## Troubleshooting

**Problem**: Wrong CAN IDs or robot doesn't drive correctly
- **Solution**: Verify `Constants.currentRobot` matches the physical robot you're deploying to

**Problem**: Modules don't align properly
- **Solution**: Run module calibration and update zero rotations in the appropriate config class

**Problem**: Turret errors on MiniBot
- **Solution**: MiniBot doesn't have a turret. The code handles this automatically by setting `turret = null`

**Problem**: Build fails after switching robot
- **Solution**: Run `./gradlew clean build` to force recompilation

## References

- MainBot configuration based on: [wavelength3572/Robot-2025](https://github.com/wavelength3572/Robot-2025)
- MiniBot configuration based on: [wavelength3572/MiniBot-2025](https://github.com/wavelength3572/MiniBot-2025)
