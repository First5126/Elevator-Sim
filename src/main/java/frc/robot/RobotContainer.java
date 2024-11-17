// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
  private final Elevator m_elevator = new Elevator();
  private final CommandXboxController m_controller = new CommandXboxController(0);
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_elevator.setDefaultCommand(m_elevator.openLoopCommand(m_controller::getLeftY));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");

  }
}
