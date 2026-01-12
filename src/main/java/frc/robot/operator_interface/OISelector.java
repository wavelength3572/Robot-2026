// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Utility class for selecting the appropriate OI implementations based on the connected joysticks.
 */
public class OISelector {
  private static String[] lastJoystickNames = new String[] {null, null, null, null, null, null};
  private static final Alert noOperatorInterfaceWarning =
      new Alert("No operator controller(s) connected.", AlertType.kWarning);
  private static final Alert nonCompetitionOperatorInterfaceWarning =
      new Alert("Non-competition operator controller connected.", AlertType.kWarning);

  private OISelector() {} // Contructor - Empty

  /**
   * Returns whether the connected joysticks have changed since the last time this method was
   * called.
   */
  public static boolean didJoysticksChange() {
    boolean joysticksChanged = false;
    for (int port = 0; port < DriverStation.kJoystickPorts; port++) {
      String name = DriverStation.getJoystickName(port);
      if (!name.equals(lastJoystickNames[port])) {
        lastJoystickNames[port] = name;
        joysticksChanged = true;
      }
    }
    return joysticksChanged;
  }

  /**
   * Instantiates and returns an appropriate OperatorInterface object based on the connected
   * joysticks.
   */
  public static OperatorInterface findOperatorInterface() {
    Integer firstPort = null;
    Integer secondPort = null;
    Integer xBoxPort = null;
    Integer interlinkDXPort = null;
    Integer keyboardPort = null;
    Integer gampadPort = null;
    Integer buttonBox1Port = null;
    Integer buttonBox2Port = null;

    for (int port = 0; port < DriverStation.kJoystickPorts; port++) {
      System.out.println("Joystick Name:[" + DriverStation.getJoystickName(port) + "]");
      if (DriverStation.getJoystickName(port).toLowerCase().contains("interlinkdx")) {
        if (interlinkDXPort == null) {
          interlinkDXPort = port;
        }
      } else if (DriverStation.getJoystickName(port).toLowerCase().contains("usb gamepad")
          || DriverStation.getJoystickName(port).toLowerCase().contains("pro")) {
        if (gampadPort == null) {
          gampadPort = port;
        }
      } else if (DriverStation.getJoystickName(port).toLowerCase().contains("a-pac")) {
        if (buttonBox1Port == null) {
          buttonBox1Port = port;
        } else if (buttonBox2Port == null) {
          buttonBox2Port = port;
        }
      } else if (DriverStation.getJoystickName(port).toLowerCase().contains("xbox")
          || DriverStation.getJoystickName(port).toLowerCase().contains("game for windows")
          || DriverStation.getJoystickName(port).toLowerCase().contains("ultimate wireless")
          || DriverStation.getJoystickName(port).toLowerCase().contains("logitech dual action")) {
        if (xBoxPort == null) {
          xBoxPort = port;
        }
      } else if (DriverStation.getJoystickName(port).toLowerCase().contains("keyboard")) {
        if (keyboardPort == null) {
          keyboardPort = port;
        }
      } else if (!DriverStation.getJoystickName(port).equals("")) {
        // Find the first 2 Joysticks NOT containing "xbox"
        // And assigned them to the first and second ports
        if (firstPort == null) {
          firstPort = port;
        } else if (secondPort == null) {
          secondPort = port;
        }
      }
    }

    if (interlinkDXPort != null && buttonBox1Port != null && buttonBox2Port != null) {
      noOperatorInterfaceWarning.set(false);
      nonCompetitionOperatorInterfaceWarning.set(false);
      return new InterLinkDXButtonBoxOI(interlinkDXPort, buttonBox1Port, buttonBox2Port);
    } else if (interlinkDXPort != null) {
      noOperatorInterfaceWarning.set(false);
      nonCompetitionOperatorInterfaceWarning.set(true);
      return new InterLinkDXOI(interlinkDXPort);
    } else if (xBoxPort != null) {
      noOperatorInterfaceWarning.set(false);
      nonCompetitionOperatorInterfaceWarning.set(true);
      return new SingleHandheldOI(xBoxPort);
    } else if (buttonBox1Port != null && buttonBox2Port != null) {
      noOperatorInterfaceWarning.set(false);
      nonCompetitionOperatorInterfaceWarning.set(true);
      return new ButtonBoxOI(buttonBox1Port, buttonBox2Port);
    } else if (keyboardPort != null) {
      noOperatorInterfaceWarning.set(false);
      nonCompetitionOperatorInterfaceWarning.set(true);
      return new KeyboardOI(keyboardPort);
    } else if (gampadPort != null) {
      noOperatorInterfaceWarning.set(false);
      nonCompetitionOperatorInterfaceWarning.set(true);
      return new GamepadOI(gampadPort);
    } else if (firstPort != null && secondPort != null) {
      noOperatorInterfaceWarning.set(false);
      nonCompetitionOperatorInterfaceWarning.set(false);
      return new DualJoysticksOI(firstPort, secondPort);
    } else {
      noOperatorInterfaceWarning.set(true);
      nonCompetitionOperatorInterfaceWarning.set(false);
      return new OperatorInterface() {};
    }
  }
}
