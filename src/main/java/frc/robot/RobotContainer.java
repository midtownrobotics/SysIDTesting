// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
  }

  private CommandXboxController controller = new CommandXboxController(0);

  private Arm arm = new Arm(controller);

  private void configureBindings() {
    // controller.b().onTrue(arm.sysIdQuasistatic(Direction.kForward));
    // controller.a().onTrue(arm.sysIdQuasistatic(Direction.kReverse));
    // controller.y().onTrue(arm.sysIdDynamic(Direction.kForward));
    // controller.x().onTrue(arm.sysIdDynamic(Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
