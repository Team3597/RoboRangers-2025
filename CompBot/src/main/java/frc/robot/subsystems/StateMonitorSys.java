// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StateMonitorSys extends SubsystemBase {

  public enum ManipulatorState {
    HOME,
    AL1,
    AL2,
    APROCESSOR,
    ANET,
    AGROUND,
    CL1,
    CL2,
    CL3,
    CL4
  }

  public enum ClimbState {
    HOME,
    READY,
    LATCHED
  }

  public static ManipulatorState manipulatorState;
  public static ClimbState climbState;

  /** Creates a new StateMonitorSys. */
  public StateMonitorSys() {
    manipulatorState = ManipulatorState.HOME;
    climbState = ClimbState.HOME;
  }

  public void setManipulatorState(ManipulatorState newState) {
    manipulatorState = newState;
  }

  public ManipulatorState getManipulatorState() {
    return manipulatorState;
  }

  public void setClimbState(ClimbState newState) {
    climbState = newState;
  }

  public ClimbState getClimbState() {
    return climbState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Manipulator and Elevator State",getManipulatorState().toString());
    SmartDashboard.putString("Climb State", getClimbState().toString());
  }
}
