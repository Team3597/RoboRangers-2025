// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ManipulatorPitchSys;
import frc.robot.subsystems.StateSys;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetScoring extends Command {
  /** Creates a new SetScoring. */

  private static StateSys stateSys;
  private static ElevatorSys elevatorSys;
  private static ManipulatorPitchSys manipulatorPitchSys;

  public SetScoring(StateSys state, ElevatorSys elevator, ManipulatorPitchSys manipulator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.stateSys = state;
    this.elevatorSys = elevator;
    this.manipulatorPitchSys = manipulator;
    addRequirements(stateSys, elevatorSys, manipulatorPitchSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
