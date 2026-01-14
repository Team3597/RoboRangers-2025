// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CLIMB;
import frc.robot.subsystems.StateSys;
import frc.robot.subsystems.StateSys.climbing;
import frc.robot.subsystems.superstructure.ClimbSys;

// relic from attempts at using pid control on the climb
// testing this blew up a versaplanetary so we switched to a dumber strategy and thus this is unused
// see {climb} command for the cro magnon control we wound up using

public class SetClimbing extends Command {
  
  private final ClimbSys climbSys;
  private final StateSys stateSys;

  private StateSys.climbing currentPos;
  private StateSys.climbing targetPos;

  private double targetPitch;

  public SetClimbing(StateSys.climbing target, StateSys state, ClimbSys climb) {
    this.targetPos = target;
    this.stateSys = state;
    this.climbSys = climb;

    addRequirements(climbSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    this.currentPos = stateSys.getClimbingState();
    this.targetPitch = targetPos.pitch();

    switch (targetPos) {
      case Home:
      if (currentPos != climbing.Latched || StateSys.climbOverride) {
        climbSys.setPitch(targetPitch);
      }
      break;
      case Ready:
      if (currentPos != climbing.Latched || StateSys.climbOverride) {
        climbSys.setPitch(targetPitch);
      }
      break;
      case Latched:
      if (currentPos == climbing.Ready || StateSys.climbOverride) {
        climbSys.setPitch(targetPitch);
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stateSys.setClimbingState(targetPos);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(climbSys.getPitch() - targetPitch) < CLIMB.DEADBAND) {
      return true;
    }
    return false;
  }
}
