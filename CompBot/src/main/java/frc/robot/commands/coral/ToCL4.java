// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ManipulatorPitchSys;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ToCL4 extends Command {
  /** Creates a new ToCL4. */

  private ElevatorSys elevatorSys;
  private ManipulatorPitchSys manipulatorPitchSys;
  
  public ToCL4(ElevatorSys elevator, ManipulatorPitchSys manipulatorPitch) {
    this.elevatorSys = elevator;
    this.manipulatorPitchSys = manipulatorPitch;
    addRequirements(elevatorSys,manipulatorPitchSys);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //elevatorSys.toCL4();
    manipulatorPitchSys.toCHigh();
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
