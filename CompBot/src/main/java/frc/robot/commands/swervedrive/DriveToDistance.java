// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToDistance extends Command {
  /** Creates a new DriveToDistance. */

  private SwerveSubsystem swerveSubsystem;
  private double distance;
  private double velocity;
  private double timeout;

  public DriveToDistance(SwerveSubsystem swerve, double d, double v, double t) {
    this.swerveSubsystem = swerve;
    this.distance = d;
    this.velocity = v;
    this.timeout = t;
    addRequirements(swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveSubsystem.driveToDistanceCommand(distance, velocity).withTimeout(timeout);
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
