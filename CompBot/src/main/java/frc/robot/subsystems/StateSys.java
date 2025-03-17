// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.MANIPULATOR;

public class StateSys extends SubsystemBase {
  /** Creates a new ScoringStateSys. */
  public enum scoring {
 // Name        Elevator pos   Manipulator pos      Extended? Coral? Intake at pos?
    Home(       ELEVATOR.HOME, MANIPULATOR.HOME,    false, true,  true),
    CL1(        ELEVATOR.CL1,  MANIPULATOR.CL1,     true,  true,  false),
    CL2(        ELEVATOR.CL2,  MANIPULATOR.CLOW,    false, true,  false),
    CL3(        ELEVATOR.CL3,  MANIPULATOR.CLOW,    false, true,  false),
    CL4(        ELEVATOR.CL4,  MANIPULATOR.CHIGH,   true,  true,  false),
    AGround(    ELEVATOR.HOME, MANIPULATOR.AGROUND, true,  false, true),
    AProcessor( ELEVATOR.HOME, MANIPULATOR.AGROUND, true,  false, false),
    AL1(        ELEVATOR.AL1,  MANIPULATOR.AREEF,   true,  false, true),
    AL2(        ELEVATOR.AL2,  MANIPULATOR.AREEF,   true,  false, true),
    ANet(       ELEVATOR.ANET, MANIPULATOR.AREEF,   true,  false, false);

    private final double elevatorHeight;
    private final double manipulatorPitch;
    private final boolean isExtended;
    private final boolean isCoral;
    private final boolean isIntake;


    scoring(double height, double pitch, boolean extended, boolean coral, boolean intake) {
      this.elevatorHeight = height;
      this.manipulatorPitch = pitch;
      this.isExtended = extended;
      this.isCoral = coral;
      this.isIntake = intake;
    }

    public double height() {
      return elevatorHeight;
    }

    public double pitch() {
      return manipulatorPitch;
    }

    public boolean extended() {
      return isExtended;
    }

    public boolean coral() {
      return isCoral;
    }

    public boolean intake() {
      return isIntake;
    }
  };

  public static boolean hasCoral;
  public static boolean hasAlgae;

  public scoring scoringState;
  
  public StateSys() {
    scoringState = scoring.Home;
    hasCoral = false;
    hasAlgae = false;
  }

  public void setScoringState(scoring newState) {
    scoringState = newState;
  }

  public scoring getScoringState() {
    return scoringState;
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Scoring State", scoringState.toString());
    // This method will be called once per scheduler run
  }

  public class StateException extends Exception {
    public StateException(String message) {
      super(message);
    }
  }
}
