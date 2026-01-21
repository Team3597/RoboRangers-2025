// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CORAL;
import frc.robot.subsystems.manipulator.CoralManipulatorSys;

// i think this class is completely unused in favor of an inline command in {CoralManipulatorSys}

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StepCoral extends Command {
  /** Creates a new StepCoral. */

  private static CoralManipulatorSys coralManipulatorSys;

  public StepCoral(CoralManipulatorSys coral) {
    this.coralManipulatorSys = coral;
    addRequirements(coralManipulatorSys);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coralManipulatorSys.manipulateCoral(CORAL.INTAKE_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralManipulatorSys.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
