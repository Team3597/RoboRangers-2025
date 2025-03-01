// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.MANIPULATOR;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ManipulatorPitchSys;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ToClear extends Command {
  /** Creates a new ToClear. */

  private static ManipulatorPitchSys manipulatorPitchSys;
  private static ElevatorSys elevatorSys;
  
  public ToClear(ManipulatorPitchSys pitch, ElevatorSys elevator) {
    this.manipulatorPitchSys = pitch;
    this.elevatorSys = elevator;

    addRequirements(manipulatorPitchSys, elevatorSys);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    manipulatorPitchSys.toHome();
    elevatorSys.toClear();;

  }

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
