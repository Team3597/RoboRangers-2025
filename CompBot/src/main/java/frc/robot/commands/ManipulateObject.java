// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeManipulatorSys;
import frc.robot.subsystems.CoralManipulatorSys;
import frc.robot.subsystems.StateMonitorSys;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManipulateObject extends Command {
  
  private final AlgaeManipulatorSys algaeManipulatorSys;
  private final CoralManipulatorSys coralManipulatorSys;

  /** Creates a new IntakeA. */
  public ManipulateObject(AlgaeManipulatorSys algaeManipulator, CoralManipulatorSys coralManipulator) {
    this.algaeManipulatorSys = algaeManipulator;
    this.coralManipulatorSys = coralManipulator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algaeManipulatorSys, coralManipulatorSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // combined 4 individual controls into one state based control. 
    // automatically does appropriate action for position; scoring is now just confirming we are aligned
    switch (StateMonitorSys.manipulatorState) {
      case AGROUND:
        algaeManipulatorSys.intakeAlgae();
        break;
      case APROCESSOR:
        algaeManipulatorSys.outtakeAlgae();
        break;
      case AL1:
        algaeManipulatorSys.intakeAlgae();
        break;
      case AL2:
        algaeManipulatorSys.intakeAlgae();
        break;
      case ANET:
        algaeManipulatorSys.outtakeAlgae();
        break;

      case HOME:
        coralManipulatorSys.intakeCoral();
        break;
      case CL1:
        coralManipulatorSys.frontOuttakeCoral();
        break;
      case CL2:
        coralManipulatorSys.frontOuttakeCoral();
        break;
      case CL3:
        coralManipulatorSys.frontOuttakeCoral();
        break;
      case CL4:
        coralManipulatorSys.backOuttakeCoral();
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

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
