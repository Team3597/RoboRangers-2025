// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GLOBAL;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ManipulatorPitchSys;
import frc.robot.subsystems.StateMonitorSys;
import frc.robot.subsystems.StateMonitorSys.ManipulatorState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ToCL4 extends Command {

  private final ElevatorSys elevatorSys;
  private final ManipulatorPitchSys manipulatorPitchSys;

  /** Creates a new ToCL4. */
  public ToCL4(ElevatorSys elevatorSys, ManipulatorPitchSys manipulatorPitchSys) {
    this.elevatorSys = elevatorSys;
    this.manipulatorPitchSys = manipulatorPitchSys;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSys, manipulatorPitchSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (GLOBAL.DEBUG_MODE) System.out.println("To CL4");
    elevatorSys.toCL4(); // elevator to cl4
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSys.moveToHeight();
    //if (elevatorSys.isClear()) manipulatorPitchSys.toCHigh(); // manipulator to chigh once elevator is clear
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    StateMonitorSys.manipulatorState = ManipulatorState.CL4;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (elevatorSys.isAtCL4() && manipulatorPitchSys.isAtCHigh()) {
      if (GLOBAL.DEBUG_MODE) System.out.println("Stopping CL4");
      return true; // ends command once system is set
    } else {
      return false;
    }
  }
}
