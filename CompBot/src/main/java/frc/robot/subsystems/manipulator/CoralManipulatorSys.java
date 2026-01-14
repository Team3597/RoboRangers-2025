// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CORAL;
import frc.robot.Constants.MANIPULATOR;

// this class is a good example of cro magnon basic voltage motor control. this only allows rough relative velocity setting,
// with setting voltage from -1 to 1 (full speed reverse or full speed forward). this cannot set an actual velocity number
// (ie 200 rpm), cannot hold the relative velocity (if something pushes against the motor to slow it, it will just slow down
// because it cannot increase the voltage to increase force and keep velocity accordingly), and cannot set position. however
// this is crazy easy to set up and works totally fine for most simple things
// the wpilib advanced controls docs refer to this as a pure feedforward controller (just control with no feedback)

// declare class to extend properties of the baseline subsystem provided by wpilib
public class CoralManipulatorSys extends SubsystemBase {
  /** Creates a new CoralManipulator. */

  /* declare 'physical' motor controller objects with their id (see in rev hardware app) and type.
  this will allow the code to send voltage signals to the motor.

  !!!!!MAKE SURE THAT YOU DECLARE BRUSHLESS MOTORS AS BRUSHLESS AND BRUSHED AS BRUSHED!!!!
  I HAVE BLOWN UP MOTORS FROM MESSING THIS UP!!! Note that this is a brushed motor rather than brushless like most other motors */
  private static SparkMax coralManipulator = new SparkMax(Constants.CAN.CORAL_MANIPULATOR, MotorType.kBrushed);

  // declare configuration profile for motor. new setup as of 2025
  // https://docs.revrobotics.com/revlib/spark/configuring-a-spark
  // https://docs.revrobotics.com/revlib/configuring-devices
  private static SparkMaxConfig coralConfig = new SparkMaxConfig();

  // note that we do not declare a digital controller as in other classes like {ElevatorSys} and {ManipulatorPitchSys}

  public CoralManipulatorSys() {

     // set up main config parameters. no pid or anything going on
    coralConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(MANIPULATOR.CORAL_AMP_LIMIT);
    // apply the config that was just defined to the sparkmax
    coralManipulator.configure(coralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // unused inline command to run the motor, rather than using a method that is accessed by a separate command file
  // this is probably broken, judging by my debugging message
  public Command runCoralCmd(double speed) {
    return new Command() {
      public void initialize() {
        coralManipulator.set(speed);
        System.out.println("kys");
      }
    };
  }

  // another inline command, running a sequential set of actions to run the motor and then quickly stop it
  // used in {ManipulateObject} to 'step' the coral forward a tiny amount; use will be better explained there
  public void stepCoralCmd() {
   //Commands.sequence(Commands.startEnd(() -> coralManipulator.set(CORAL.INTAKE_SPEED), () -> coralManipulator.stopMotor()).withTimeout(3));
   System.out.println("stepCoral"); 
   Commands.sequence(
      Commands.runOnce(() -> coralManipulator.set(1)),
      Commands.waitSeconds(0.0),
      Commands.runOnce(() -> coralManipulator.stopMotor()),
      Commands.runOnce(() -> System.out.println("end"))
    ).schedule();
       //runCoralCmd(1).andThen(Commands.waitSeconds(3)).andThen(Commands.runOnce(() -> coralManipulator.stopMotor())).schedule();
       
  }

  // basic method to run motor at any speed. notice you just set a speed -1 to 1
  public void manipulateCoral(double speed) {
    coralManipulator.set(speed);
  }

  // method to initially intake coral and then stop intaking once a beambreak in the manipulator is blocked by the coral
  public void intakeCoral() {
    // check if the system does not have coral. beambreak backend is handled in {AlgaeManipulatorSys} for both algae and coral
    // because the beambreaks are wired into the algae sparkmax as limit switches instead of running new wires all the way to
    // the roborio digital inputs
   if (!AlgaeManipulatorSys.hasCoral) {
      // intake coral if it isn't already in the system
      coralManipulator.set(CORAL.INTAKE_SPEED);
   } else {
      // once the system has coral stop intaking
      coralManipulator.stopMotor();
   }

  }



  // for scoring L1-L3
  // basic method to feed coral out the front of the manipulator
  public void frontOuttakeCoral() {
    coralManipulator.set(CORAL.FRONT_OUTTAKE_SPEED);
  }

  // for scoring L4
  // basic method to feed coral out the back of the manipulator
  public void backOuttakeCoral() {
    coralManipulator.set(-CORAL.BACK_OUTTAKE_SPEED);
  }

  // always need a stop method. setting the motor speed sets it indefinitely until a stop is called
  public void stop() {
    coralManipulator.stopMotor();
  }
}
