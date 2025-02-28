// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GLOBAL;
import frc.robot.Constants.MOTION;
import frc.robot.subsystems.CoralManipulatorSys;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreC extends Command {

  private final CoralManipulatorSys coralManipulatorSys;

  /** Creates a new ScoreC. */
  public ScoreC(CoralManipulatorSys coralManipulatorSys) {
    this.coralManipulatorSys = coralManipulatorSys;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coralManipulatorSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (GLOBAL.manipulatorPos.equals("CL4")) { // checks if manipulator is at CL4
      coralManipulatorSys.backOuttakeCoral(MOTION.CORAL_BACK_OUTTAKE_SPEED);
    } else {
      coralManipulatorSys.frontOuttakeCoral(MOTION.CORAL_FRONT_OUTTAKE_SPEED);
    }
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
