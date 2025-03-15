// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StateSys extends SubsystemBase {
  /** Creates a new ScoringStateSys. */
  public enum scoring {
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

  public scoring scoringState;
  

  
  public StateSys() {
    scoringState = scoring.Home;
  }

  public void setState(scoring newState) {
    scoringState = newState;
  }

  public scoring getState() {
    return scoringState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
