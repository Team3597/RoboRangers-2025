// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CLIMB;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.MANIPULATOR;

// this subsystem tracks various properties of the robot as a set of 'states'
// instead of making a command for every possible combination of property changes or 
// individual commands for each change, we run a singular {@link SetScoring} command
// saying 'go to this state' and the command reads the properties of this state 

public class StateSys extends SubsystemBase {
  /** Creates a new ScoringStateSys. */

  /* this enum (collection of constants) holds all of the properties of each scoring state.
    each enum is an instance of the scoring subclass defined below with parameters for
    elevator and manipulator positions, whether the manipulator is extended outward, whether
    the robot is handling coral (assumes handling algae if false), and whether the robot should
    be intaking (assumes outtaking if false) */
  public enum scoring {
 // Name        Elevator pos   Manipulator pos          Extended?       Coral?       Intake at pos?
    Home(       ELEVATOR.HOME, MANIPULATOR.HOME,        false, true,  true),
    CL1(        ELEVATOR.CL1,  MANIPULATOR.CL1,         true,  true,  false),
    CL2(        ELEVATOR.CL2,  MANIPULATOR.CLOW,        false, true,  false),
    CL3(        ELEVATOR.CL3,  MANIPULATOR.CLOW,        false, true,  false),
    CL4(        ELEVATOR.CL4,  MANIPULATOR.CHIGH,       true,  true,  false),
    AGround(    ELEVATOR.HOME, MANIPULATOR.AGROUND,     true,  false, true),
    AProcessor( ELEVATOR.HOME, MANIPULATOR.APROCESSOR,  true,  false, false),
    AL1(        ELEVATOR.AL1,  MANIPULATOR.AREEF,       true,  false, true),
    AL2(        ELEVATOR.AL2,  MANIPULATOR.AREEF,       true,  false, true),
    ANet(       ELEVATOR.ANET, MANIPULATOR.ANET,        true,  false, false);

    private final double elevatorHeight;
    private final double manipulatorPitch;
    private final boolean isExtended;
    private final boolean isCoral;
    private final boolean isIntake;

    public boolean ownsCoral;

    // constructor used in enum, setting each subclass object in the enum to have the parameters 
    // passed into the constructor 
    scoring(double height, double pitch, boolean extended, boolean coral, boolean intake) {
      this.elevatorHeight = height;
      this.manipulatorPitch = pitch;
      this.isExtended = extended;
      this.isCoral = coral;
      this.isIntake = intake;
    }

    // getters to read things about the robot's state
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

    public boolean algae() {
      return !isCoral;
    }
  };

  // similar to before, but for climbing
  public enum climbing {
    Home(CLIMB.HOME),
    Ready(CLIMB.READY),
    Latched(CLIMB.LATCHED);

    private double climbPitch;

    climbing(double pitch) {
      this.climbPitch = pitch;
    }

    public double pitch() {
      return climbPitch;
    }
  }


  public static boolean climbOverride;
  public climbing climbingState;
  public scoring scoringState;
  
  // constructor for statesys object, initializing to home states
  public StateSys() {
    scoringState = scoring.Home;
    climbingState = climbing.Home;
    climbOverride = false;
  }

  // setter and getter methods to set and get current scoring and climbing states 
  public void setScoringState(scoring newState) {
    scoringState = newState;
  }

  public scoring getScoringState() {
    return scoringState;
  }

  public void setClimbingState(climbing newState) {
    climbingState = newState;
  }

  public climbing getClimbingState() {
    return climbingState;
  }

  // put the current state on the dashboard (largely for debugging)
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
