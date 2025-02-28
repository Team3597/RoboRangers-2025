// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.GLOBAL;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ManipulatorPitchSys;
import frc.robot.subsystems.StateMonitorSys;
import frc.robot.subsystems.StateMonitorSys.ManipulatorState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ToANet extends Command {

  private final ElevatorSys elevatorSys;
  private final ManipulatorPitchSys manipulatorPitchSys;

  /** Creates a new ToANet. */
  public ToANet(ElevatorSys elevatorSys, ManipulatorPitchSys manipulatorPitchSys) {
    this.elevatorSys = elevatorSys;
    this.manipulatorPitchSys = manipulatorPitchSys;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSys, manipulatorPitchSys);
  }

  // Called when the command is initially scheduled.
  @Override
 public void initialize() {
    elevatorSys.toANet(); //start moving
    while (elevatorSys.GetElevatorPosition() < Constants.ELEVATOR.CLEAR - Constants.ELEVATOR.DEADBAND) {} // waits until elevator is at clear height
    manipulatorPitchSys.toANet(); //only move out once clear
    if (GLOBAL.DEBUG_MODE) {
      System.out.println("ToANet");
    }
    //reduce drive speed?
    StateMonitorSys.manipulatorState = ManipulatorState.ANET;
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
