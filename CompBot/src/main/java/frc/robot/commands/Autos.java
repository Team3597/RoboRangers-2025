// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSys;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.lang.UnsupportedOperationException;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSys subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  // public static Command driveBackwardsAuto(SwerveSubsystem swerveSubsystem) {
  //   return Commands.sequence(
  //     new SequentialCommandGroup(
  //       new driveToDistanceCommand()
  //     )
  //   )
  // }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
