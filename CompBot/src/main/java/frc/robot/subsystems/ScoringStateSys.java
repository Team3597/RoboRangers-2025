// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ScoringStateSys extends SubsystemBase {
  /** Creates a new ScoringStateSys. */
  private enum states {
    Home,
    CL1,
    CL2,
    CL3,
    CL4,
    AGround,
    AL1,
    AL2,
    ANet
  };

  
  public ScoringStateSys() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
