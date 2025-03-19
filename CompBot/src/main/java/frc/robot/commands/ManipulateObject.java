// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeManipulatorSys;
import frc.robot.subsystems.CoralManipulatorSys;
import frc.robot.subsystems.StateSys;
import frc.robot.subsystems.StateSys.scoring;

public class ManipulateObject extends Command {

  private final StateSys stateSys;
  private final CoralManipulatorSys coralManipulatorSys;
  private final AlgaeManipulatorSys algaeManipulatorSys;

  private scoring currentPos;
  
  public ManipulateObject(StateSys state, CoralManipulatorSys coral, AlgaeManipulatorSys algae) {
    this.stateSys = state;
    this.coralManipulatorSys = coral;
    this.algaeManipulatorSys = algae;
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.currentPos = stateSys.getScoringState();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //coral
    if (currentPos.coral()) {
      if (currentPos.intake()) {
        coralManipulatorSys.intakeCoral();
      } else {
        if (currentPos == scoring.CL4) {
          coralManipulatorSys.backOuttakeCoral();
        } else {
          coralManipulatorSys.frontOuttakeCoral();
        }
      }
    //algae
    } else {
      if (currentPos.intake()) {
        algaeManipulatorSys.intakeAlgae();
      } else {
        algaeManipulatorSys.outtakeAlgae();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeManipulatorSys.stop();
    coralManipulatorSys.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
