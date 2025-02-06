// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Arm;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();

    DataLogManager.start();
  }

  private CommandXboxController controller = new CommandXboxController(0);

  private Arm arm = new Arm(controller);

  private void configureBindings() {
    // controller.b().whileTrue(arm.sysIdQuasistatic(Direction.kForward));
    // controller.a().whileTrue(arm.sysIdQuasistatic(Direction.kReverse));
    // controller.y().whileTrue(arm.sysIdDynamic(Direction.kForward));
    // controller.x().whileTrue(arm.sysIdDynamic(Direction.kReverse));

    controller.povDown().onTrue(arm.setTargetPositionCommand(Rotation2d.fromDegrees(-90)));
    controller.povDownRight().onTrue(arm.setTargetPositionCommand(Rotation2d.fromDegrees(-45)));
    controller.povRight().onTrue(arm.setTargetPositionCommand(Rotation2d.fromDegrees(0)));
    controller.povUpRight().onTrue(arm.setTargetPositionCommand(Rotation2d.fromDegrees(45)));
    controller.povUp().onTrue(arm.setTargetPositionCommand(Rotation2d.fromDegrees(90)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
