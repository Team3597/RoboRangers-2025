// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.MANIPULATOR;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ManipulatorPitchSys;
import frc.robot.subsystems.StateSys;
import frc.robot.subsystems.StateSys.scoring;

// nomenclature change: position now only refers to overall scoring position; ie L1
// height and pitch then refer to the components of this for the elevator and manipulator

public class SetScoring extends Command {

  private final StateSys stateSys;
  private final ElevatorSys elevatorSys;
  private final ManipulatorPitchSys manipulatorPitchSys;

  private StateSys.scoring currentPos;
  private StateSys.scoring targetPos;

  private double targetHeight;
  private double targetPitch;

  public SetScoring(StateSys.scoring target, StateSys state, ElevatorSys elevator, ManipulatorPitchSys manipulator) {
    this.targetPos = target;
    this.stateSys = state;
    this.elevatorSys = elevator;
    this.manipulatorPitchSys = manipulator;
    addRequirements(stateSys, elevatorSys, manipulatorPitchSys);
  }

  @Override
  public void initialize() {
    this.currentPos = stateSys.getScoringState();
    this.targetHeight = targetPos.height();
    this.targetPitch = targetPos.pitch();
    
    
      // if manipulator moving through clear
    if ((targetPos == scoring.AGround && currentPos == scoring.Home) || (targetPos == scoring.Home && currentPos == scoring.AGround) || (targetPos == scoring.AProcessor && currentPos == scoring.Home) || (targetPos == scoring.Home && currentPos == scoring.AProcessor)) {
      // go to clear before anything else
      elevatorSys.setProfile(ELEVATOR.CLEAR);
    } else {
      // otherwise go right to final height
      elevatorSys.setProfile(targetHeight);
    }
  }

  @Override
  public void execute() {
    elevatorSys.followProfile();
    // if elevator is above clear move manipulator
    if (elevatorSys.getHeight() > ELEVATOR.CLEAR - ELEVATOR.DEADBAND) {
      if (StateSys.hasAlgae && targetPos.coral()) {
      } else {
        manipulatorPitchSys.setPitch(targetPitch);
      }
    }
    // once at final position move elevator to target
    if (Math.abs(manipulatorPitchSys.getPitch() - targetPitch) < MANIPULATOR.DEADBAND) {
      elevatorSys.setProfile(targetHeight);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stateSys.setScoringState(targetPos);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(elevatorSys.getHeight() - targetHeight) < ELEVATOR.END_DEADBAND
        && Math.abs(manipulatorPitchSys.getPitch() - targetPitch) < MANIPULATOR.END_DEADBAND) {
          return true;
    }
    return false;
  }
}
