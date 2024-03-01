// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.controllers.DriverController;
import frc.robot.controllers.OperatorController;

@SuppressWarnings("unused")
public class RobotContainer {

  private DriverController m_driverController;
  private OperatorController m_operatorController;

  public RobotContainer() {
       
    m_driverController = new DriverController(false, false, false, false);
    m_operatorController = new OperatorController(false);

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
