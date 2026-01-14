// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.MANIPULATOR;


/* this will be more heavily commented than other subsystems as it is a good example of complex motor control.
  this type of motor control involves an ABSOLUTE encoder, where zero is set to a specific position in rev
  hardware client that persists through reinitializations. the encoder reading cannot exceed a full rotation
  (although you might be able to configure it to allow this) and instead wraps to zero or within a set
  range; thus absolute encoders should be used for things that do not make a full rotation. the encoder used
  here is a through bore encoder mounted DIRECTLY to the rotating drive shaft for the manipulator pitch. the
  more direct the better (avoids backlash issues and sources of error), and not gearing to artificially increase
  the encoder resolution doesn't matter here since through bore encoders have a very high resolution of 8192 counts
  per revolution
  
  with an absolute encoder the bot doesn't necessarily have to initialize with the moving components in the home
  position. however, IT MUST BE FREE TO MOVE TO THE HOME POSITION as on startup it will drive to home immediately
  and collide or fight against anything in the way. for example, here you would not clamp the manipulator in a raised
  position and then turn the robot on because it will fight the clamp and stall the motor. 
  */

// declare class to extend properties of the baseline subsystem provided by wpilib
public class ManipulatorPitchSys extends SubsystemBase {

  /* declare 'physical' motor controller objects with their id (see in rev hardware app) and type.
  this will allow the code to send voltage signals to the motor.

  !!!!!MAKE SURE THAT YOU DECLARE BRUSHLESS MOTORS AS BRUSHLESS AND BRUSHED AS BRUSHED!!!!
  I HAVE BLOWN UP MOTORS FROM MESSING THIS UP!!! */
  private static SparkMax manipulatorPitch = new SparkMax(CAN.MANIPULATOR_PITCH, MotorType.kBrushless);

/*declare 'digital' motor controller object which will translate desired positions into direct
  signals to the motor controller using PID control. direct motor control (like in {CoralManipulatorSys})
  only allows motor velocity to be roughly controlled, while PID allows exact position and velocity control. 
  there are some great docs for this online that you should read before trying this yourself.
  highly recommend you read through the entirety of this: 
  https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/index.html */
  private static SparkClosedLoopController pitchController;

  // declare configuration profile for motor. new setup as of 2025
  // https://docs.revrobotics.com/revlib/spark/configuring-a-spark
  // https://docs.revrobotics.com/revlib/configuring-devices
  private static SparkMaxConfig pitchConfig = new SparkMaxConfig();

  public ManipulatorPitchSys() {

    // initialize digital motor controller object. could also be done in the declaration above like in {ElevatorSys}
    // getClosedLoopController() gets the built in controller setup inside the spark max. does not specify
    // the physical encoder device yet.
    pitchController = manipulatorPitch.getClosedLoopController();

    // set up main config parameters, including the PID tuning parameters.
    // once you've read the wpilib general PID info this explains the implementation specifically
    // for revlib: https://docs.revrobotics.com/revlib/spark/closed-loop

    // note that this setup does not use a motion profile line in {ElevatorSys}. this type of direct rotation generally
    // shouldn't need a profile, but as discussed in elevator, try direct PID first and only add profiles if it sucks 
    pitchConfig.closedLoop
      /*!!!! HERE IS WHERE THE ENCODER TYPE IS SET!! COMMON POINT OF ERROR IS SETTING THIS WRONG BECAUSE REV'S NAMING SUCKS!!!!
      kAbsoluteEncoder reads the ABSOLUTE encoder PLUGGED INTO THE SPARK MAX DATA PORT (explained at the top of this file around line 25).
      see {ElevatorSys} for PID with a relative encoder */
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .p(MANIPULATOR.PID.P)
      .i(MANIPULATOR.PID.I)
      .d(MANIPULATOR.PID.D)
      .outputRange(MANIPULATOR.PID.MIN, MANIPULATOR.PID.MAX)
      // here is where you might be able to make the encoder read more than 1 rotation
      .positionWrappingEnabled(true)
      // set the range outside of which wrapping happens. make sure the mechanism physically can't be pushed outside the wrapping
      // range, as it will attempt to drive backwards from expected to try and return to the desired position and crash
      // this was what the lebron tests were dealing with
      .positionWrappingInputRange(MANIPULATOR.PID.WRAP_MIN, MANIPULATOR.PID.WRAP_MAX);
    pitchConfig
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      // set a low amp limit first time you run stuff like this so if it crashes and stalls its fine
      .smartCurrentLimit(MANIPULATOR.AMP_LIMIT);
    // apply the config that was just defined to the sparkmax
    manipulatorPitch.configure(pitchConfig, ResetMode.kResetSafeParameters, null);
    // on initialization set the manipulator to the home position
    // we pass a position value and specify it is a position value (not a velocity or voltage value)
    // we do not give a feedforward value like in {ElevatorSys} because while this has to fight gravity the force
    // is not constant as the manipulator changes
    pitchController.setReference(MANIPULATOR.HOME, ControlType.kPosition);
  }

  // method to set the pitch to a passed value, setting the reference as above
  public void setPitch(double pitch) {
    pitchController.setReference(pitch, ControlType.kPosition);
    // send the target to the dashboard for tuning and debugging. tune the pid controller by graphing the target and real
    // encoder reading together in advantagescope and follow instructions in wpi advanced controls docs
    SmartDashboard.putNumber("Manipulator Target", pitch);
  }

  // unused checker to see if manipulator has reached the target pitch (relic of the old sensor based state machine)
  public boolean atPitch(double pitch) {
    if (Math.abs(getPitch() - pitch) < MANIPULATOR.DEADBAND) {
      return true;
    } 
    return false;
  }

  // unused old pitch setter not sure what this was from
  public void setReference(double pitch) {
    pitchController.setReference(pitch + MANIPULATOR.MANIPULATOR_PIVOT_OFFSET, ControlType.kPosition);
  }

  // getter method to read the encoder. note getAbsoluteEncoder to get the object for the absolute encoder plugged 
  // into the data port and then getPosition called on that encoder object
  public double getPitch() {
    return manipulatorPitch.getAbsoluteEncoder().getPosition();
  }

  // send data to the dashboard for debugging and tuning
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Manipulator Encoder", getPitch());
    // this one has some stupid unit conversion stuff. generally avoid
    SmartDashboard.putNumber("Manipulator Neo Encoder", 0.48 * (manipulatorPitch.getEncoder().getPosition()/33.64));
    // graphing current when tuning pid is also a good idea; try and drive response to be faster until it either
    // becomes unstable (seen from encoder position) or the current peaks and the motor is at its limit
    SmartDashboard.putNumber("Manipulator Current", manipulatorPitch.getOutputCurrent());
  }

}
