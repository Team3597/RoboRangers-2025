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

    this.currentPos = stateSys.getScoringState();
    this.targetHeight = targetPos.height();
    this.targetPitch = targetPos.pitch();
  }

  @Override
  public void initialize() {
    System.out.println("new SetScoring with el and manip " + targetHeight + targetPitch + " to pos " + targetPos.toString());
      // if manipulator moving through clear
    if ((targetPos == scoring.AGround && currentPos == scoring.Home) || (targetPos == scoring.Home && currentPos == scoring.AGround)) {
      // go to clear before anything else
      System.out.println("moving through clear, setting elevator to clear");
      elevatorSys.setProfile(ELEVATOR.CLEAR);
    } else {
      // otherwise go right to final height
      System.out.println("not clearing, setting elevator to " + targetHeight);
      elevatorSys.setProfile(targetHeight);
    }
  }

  @Override
  public void execute() {
    elevatorSys.followProfile();
    // if elevator is above clear move manipulator
    if (elevatorSys.getHeight() > ELEVATOR.CLEAR - ELEVATOR.DEADBAND) {
      System.out.println("elevator is at clear");
      if (StateSys.hasAlgae && targetPos.coral()) {
        System.out.println("Cannot move to coral positions with algae");
      } else {
        manipulatorPitchSys.setPitch(targetPitch);
        System.out.println("setting manipulator to " + targetPitch);
      }
    }
    // once at final position move elevator to target
    if (Math.abs(manipulatorPitchSys.getPitch() - targetPitch) < MANIPULATOR.DEADBAND) {
      elevatorSys.setProfile(targetHeight);
      System.out.println("manipulator is at final, setting elevator to " + targetHeight);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stateSys.setScoringState(targetPos);
    System.out.println("setting state to " + targetPos.toString());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("distance from end is " + Math.abs(elevatorSys.getHeight() - targetHeight));
    System.out.println("distance from pitch is " + Math.abs(manipulatorPitchSys.getPitch() - targetPitch));
    if (Math.abs(elevatorSys.getHeight() - targetHeight) < ELEVATOR.END_DEADBAND
        && Math.abs(manipulatorPitchSys.getPitch() - targetPitch) < MANIPULATOR.END_DEADBAND) {
          return true;
    }
    return false;
  }
}
