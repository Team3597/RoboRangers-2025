// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.ClimbSys;

// this is the super simple climb command we actually wound up using. good example of basic commands

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

// declare class to extend properties of the baseline command provided by wpilib
public class climb extends Command {

  // declare uninitialized instance of relevant subsystem classe
  private static ClimbSys climbSys;
  
  // declare uninitialized variable for the speed to run the motors at
  private double speed;

  // command constructor, accepting the speed to run the climb at and the master climb subsystem from {RobotContainer}
  public climb(double speed, ClimbSys climb) {
    // initialize subsystem object in this file to master for subsystem methods to be called on
    this.climbSys = climb;
    // initalize speed to the speed passed through the constructor
    this.speed = speed;
    addRequirements(climbSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // call the method in {ClimbSys} to set the motors to the speed passed to the command
    climbSys.climb(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // call the method in {ClimbSys} to stop the motors
    climbSys.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // always returns false, meaning the command only ends when the button bindings end it in {RobotContainer}
    // since it is bound as WhileTrue, it ends when the button controlling it is depressed
    return false;
  }
}
