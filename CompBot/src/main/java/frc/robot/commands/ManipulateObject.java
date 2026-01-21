// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StateSys;
import frc.robot.subsystems.StateSys.scoring;
import frc.robot.subsystems.manipulator.AlgaeManipulatorSys;
import frc.robot.subsystems.manipulator.CoralManipulatorSys;

/*this command handles all the logic for intaking and outtaking algae and coral, collectively referred to as manipulating.
  this reads the current state to manipulate the correct object in the correct direction automatically from a single button,
  rather than needing four separate buttons for both objects and directions */

// declare class to extend properties of the baseline command provided by wpilib
public class ManipulateObject extends Command {

  // declare uninitialized instances of relevant subsystem classes
  private final StateSys stateSys;
  private final CoralManipulatorSys coralManipulatorSys;
  private final AlgaeManipulatorSys algaeManipulatorSys;

  // declare uninitialized object of the state enum from {StateSys} with all the data for scoring
  // states and whether each state should intake or outtake coral or algae
  private scoring currentPos;
  
  // command constructor, accepting master objects of subsystems from {RobotContainer}
  public ManipulateObject(StateSys state, CoralManipulatorSys coral, AlgaeManipulatorSys algae) {
    // initialize subsystem objects in this file to masters for subsystem methods to be called on
    this.stateSys = state;
    this.coralManipulatorSys = coral;
    this.algaeManipulatorSys = algae;
    // has no requirements. this is because we want this command to be able to run at the same time as other commands accessing
    // the relevant subsystems, otherwise hitting the manipulate button while changing scoring positions would either block the
    // manipulate or stop the elevator and manipulator movement
    addRequirements();
  }

  // the rest of this file concerns the various steps of the command execution process

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // read the current state of the system from the master {StateSys} object
    this.currentPos = stateSys.getScoringState();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if the current state involves manipulating coral
    if (currentPos.coral()) {
      // if the current state involves intaking
      if (currentPos.intake()) {
        // intake coral
        coralManipulatorSys.intakeCoral();
      // if the current state does not involve intaking (involves outtaking)
      } else {
        // if the current state is level 4 coral, where the manipulator is fully raised and thus
        // coral is scored out the back of the manipulator 
        if (currentPos == scoring.CL4) {
          // outtake through the back of the manipulator
          coralManipulatorSys.backOuttakeCoral();
        // if the current state is not level 4 coral
        } else {
          // outtake through the front of the manipulator
          coralManipulatorSys.frontOuttakeCoral();
        }
      }
    // if the current state does not involve manipulating coral (involves algae)
    } else {
      // if the current state involves intaking
      if (currentPos.intake()) {
        // intake algae
        algaeManipulatorSys.intakeAlgae();
      // if the current state does not involve intaking (involves outtaking)
      } else {
        // outtake algae
        algaeManipulatorSys.outtakeAlgae();
      }
    }

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stop the algae motor
    algaeManipulatorSys.stop();
    // if the robot has coral in the manipulator and is at home
    if (AlgaeManipulatorSys.hasCoral && currentPos == scoring.Home) {
      // step the coral forward a little. this is needed because the limit switch triggers before the coral is fully in the
      // manipulator, so it has to be moved forward to the correct position
      coralManipulatorSys.stepCoralCmd();
    // if the robot does not have coral in the manipulator or is not at home
    } else {
      // stop the coral motor
      coralManipulatorSys.stop();
    }
    // if the robot is at an algae intaking position
    if (currentPos.algae() && currentPos.intake()) {
      // run the algae motor slowly to constantly pull algae inward and hold it
      algaeManipulatorSys.holdAlgae();
    // if the robot is not at an algae intaking position
    } else {
      // stop the algae motor
      algaeManipulatorSys.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if the robot is at home (coral intaking position)
    if (currentPos == scoring.Home) {
      // if there is coral in the manipulator
      if (AlgaeManipulatorSys.hasCoral) {
        // end command
        return true;
      }
    // if the robot is not at home and is at an algae intaking position
    } else if (currentPos.algae() && currentPos.intake()) {
      // if there is algae in the manipulator
      if (AlgaeManipulatorSys.hasAlgae) {
        // end command
        return true;
      }
    }
    // if the robot does not have coral or algae when it should keep the command running
    // to keep trying to intake until it has what it needs
    return false;
  }
}
