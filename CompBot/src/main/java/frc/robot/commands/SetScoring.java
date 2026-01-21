// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.MANIPULATOR;
import frc.robot.subsystems.StateSys;
import frc.robot.subsystems.StateSys.scoring;
import frc.robot.subsystems.manipulator.AlgaeManipulatorSys;
import frc.robot.subsystems.manipulator.ManipulatorPitchSys;
import frc.robot.subsystems.superstructure.ElevatorSys;

/*this command handles all the logic for moving the scoring superstructure, including the elevator
  and manipulator. this moves the superstructure from the current state to a state passed through
  button bindings in {RobotContainer}, and updates the current state to the new one once it is reached

  important nomenclature: position now only refers to overall scoring position; ie L1
  height and pitch then refer to the components of this for the elevator and manipulator */

// declare class to extend properties of the baseline command provided by wpilib
public class SetScoring extends Command {

  // declare uninitialized instances of relevant subsystem classes
  private final StateSys stateSys;
  private final ElevatorSys elevatorSys;
  private final ManipulatorPitchSys manipulatorPitchSys;

  // declare two uninitialized objects of the state enum from {StateSys} with all the data for scoring
  // states and their component superstructure positions
  private StateSys.scoring currentPos;
  private StateSys.scoring targetPos;

  // declare uninitialized target component positions
  private double targetHeight;
  private double targetPitch;

  // command constructor, accepting target state from button binding and master objects of subsystems from {RobotContainer}
  public SetScoring(StateSys.scoring target, StateSys state, ElevatorSys elevator, ManipulatorPitchSys manipulator) {
    // initialize target state in this file to the one passed from button binding
    this.targetPos = target;
    // initialize subsystem objects in this file to masters for subsystem methods to be called on
    this.stateSys = state;
    this.elevatorSys = elevator;
    this.manipulatorPitchSys = manipulator;
    // specifying requirements prevents conflicts from multiple commands accessing the same subsystem simultaneously.
    // if another command is run while this one runs that also has one of these requirements, it will resolve the conflict
    // by choosing one or the other. you can set which takes precedence
    addRequirements(elevatorSys, manipulatorPitchSys);
  }

  // the rest of this file concerns the various steps of the command execution process

  // initialize runs once the instant the command is scheduled
  @Override
  public void initialize() {
    // set local current state to the current state from the master {StateSys} object
    // note naming--stateSys is the object of the StateSys class
    this.currentPos = stateSys.getScoringState();
    // set desired superstructure component positions to those from the target state
    this.targetHeight = targetPos.height();
    this.targetPitch = targetPos.pitch();
    
    
    // the manipulator cannot directly move outward from home without the elevator raising, as there is a mechanical interference
    // to address this we check if the manipulator is moving in such a way by checking if it is moving through the possible pairs
    // of current and target states that involve this movement
    if ((targetPos == scoring.AGround && currentPos == scoring.Home) 
     || (targetPos == scoring.Home && currentPos == scoring.AGround) 
     || (targetPos == scoring.AProcessor && currentPos == scoring.Home) 
     || (targetPos == scoring.Home && currentPos == scoring.AProcessor)) {
      // if the movement will cause a crash, raise the elevator to a clear position before anything else
      elevatorSys.setProfile(ELEVATOR.CLEAR);
    } else {
      // otherwise go right to final height
      elevatorSys.setProfile(targetHeight);
    }
  }

  // execute runs periodically while the command is scheduled
  @Override
  public void execute() {
    // run the elevator to constantly step along its trapezoidal profile, see {ElevatorSys} for more info
    elevatorSys.followProfile();
    // if elevator is at or above the clear position and is thus free to move without crashing
    // note deadband--very important for anything like this
    if (elevatorSys.getHeight() > ELEVATOR.CLEAR - ELEVATOR.DEADBAND) {
        // drive the manipulator to the target pitch
        manipulatorPitchSys.setPitch(targetPitch);
    }
    // once manipulator is at final position move elevator to target. if the elevator didn't need to raise to clear, the profile
    // is already set to end at the target height so this does nothing. if the elevator did need to move to clear, this moves
    // it back down to where it should end now that the manipulator is done moving
    if (Math.abs(manipulatorPitchSys.getPitch() - targetPitch) < MANIPULATOR.DEADBAND) {
      elevatorSys.setProfile(targetHeight);
    }
  }

  // runs once the instant the command ends
  @Override
  public void end(boolean interrupted) {
    // update the state to the new state now that it has been reached
    stateSys.setScoringState(targetPos);
  }

  // allows the end of the command to be set by returning true
  @Override
  public boolean isFinished() {
    // check if the elevator and manipulator are at the target positions
    if (Math.abs(elevatorSys.getHeight() - targetHeight) < ELEVATOR.END_DEADBAND
        && Math.abs(manipulatorPitchSys.getPitch() - targetPitch) < MANIPULATOR.END_DEADBAND) {
          // return true to end the command
          return true;
    }
    // otherwise keep running the command until targets are reached
    return false;
  }
}
