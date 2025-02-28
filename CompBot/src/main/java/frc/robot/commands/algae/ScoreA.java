// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MOTION;
import frc.robot.subsystems.AlgaeManipulatorSys;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreA extends Command {

  private final AlgaeManipulatorSys algaeManipulatorSys;

  /** Creates a new ScoreA. */
  public ScoreA(AlgaeManipulatorSys algaeManipulatorSys) {
    this.algaeManipulatorSys = algaeManipulatorSys;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algaeManipulatorSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    algaeManipulatorSys.outtakeAlgae(MOTION.ALGAE_OUTTAKE_RPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeManipulatorSys.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
