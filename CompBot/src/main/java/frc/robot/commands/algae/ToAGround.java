// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.GLOBAL;
import frc.robot.Constants.MANIPULATOR;
import frc.robot.commands.ToClear;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ManipulatorPitchSys;
import frc.robot.subsystems.StateMonitorSys;
import frc.robot.subsystems.StateMonitorSys.ManipulatorState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ToAGround extends Command {

  private final ElevatorSys elevatorSys;
  private final ManipulatorPitchSys manipulatorPitchSys;

  /** Creates a new ToAGround. */
  public ToAGround(ElevatorSys elevatorSys, ManipulatorPitchSys manipulatorPitchSys) {
    this.elevatorSys = elevatorSys;
    this.manipulatorPitchSys = manipulatorPitchSys;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSys, manipulatorPitchSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (StateMonitorSys.manipulatorState == ManipulatorState.HOME) {
      // elevatorSys.toClear(); //move up
      // if (!GLOBAL.DISABLE_ELEVATOR) {
      //  // while (elevatorSys.GetElevatorPosition() < ELEVATOR.CLEAR - ELEVATOR.DEADBAND) {} // waits until elevator is at clear height
      // }

      new SequentialCommandGroup(
        new ToClear(manipulatorPitchSys,elevatorSys),
        new InstantCommand(() -> manipulatorPitchSys.toAGround()),
        new InstantCommand(() -> elevatorSys.toHome())
      );

      manipulatorPitchSys.toAGround(); //flip out
      //while (manipulatorPitchSys.getEncoder() < MANIPULATOR.AGROUND - MANIPULATOR.DEADBAND) {} // waits until manipulator is at safe pitch
      elevatorSys.toHome(); //move back down
      if (GLOBAL.DEBUG_MODE) {
        System.out.println("ToAGround w/ clear");
      }
    } else {
      manipulatorPitchSys.toAGround();
      elevatorSys.toHome();
      if (GLOBAL.DEBUG_MODE) {
        System.out.println("ToAGround w/o clear");
      }
    }
    StateMonitorSys.manipulatorState = ManipulatorState.AGROUND;
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
