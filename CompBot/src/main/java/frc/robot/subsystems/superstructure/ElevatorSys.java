// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CAN;
import frc.robot.Constants.ELEVATOR;

/* this will be more heavily commented than other subsystems as it is a good example of complex motor control.
  this type of motor control involves a RELATIVE encoder, where zero is set wherever the rotating object starts
  at robot initialization and the reading increases or decreasing relative to this point. the main benefit of
  relative control is the encoder value does not wrap back to zero when the rotating object makes more than one
  rotation, which is important here as the encoder being read is built into the neo and directly reads the output
  shaft before any reduction and thus makes many full rotations. reading the motor output instead of the final
  rotating object is generally a bad idea, but works here because 1. reading the final elevator chain sprocket
  still isn't directly reading the linear motion of the elevator and 2. system backlash is largely removed since
  gravity keeps the elevator on one end of the slop travel zone. additionally while the built in neo encoder has
  very low precision (42 counts per revolution), it is fine here since the motor is geared down and the overall travel 
  is large. also the elevator position can be pretty loose, like +- 1 inch 

  see {CoralManipulatorSys} for simple caveman motor control and {ManipulatorPitchSys} for ABSOLUTE encoding
  
  WHEN USING RELATIVE ENCODERS MAKE SURE THE BOT ALWAYS INITIALIZES IN THE CORRECT HOME!! for example, if you
  turn on the bot with the elevator in the middle of its travel, that becomes zero. then when you tell it to move to
  CL4 it will simply move CL4 height units upward, NOT CL4 units from the bottom of the elevator travel. this will
  make it crash into the top of the elevator travel at warp speed (keep in mind this elevator is crazy heavy and
  crazy fast) and almost certainly break something
  */

// declare class to extend properties of the baseline subsystem provided by wpilib
public class ElevatorSys extends SubsystemBase {

  /* declare 'physical' motor controller objects with their id (see in rev hardware app) and type.
    this will allow the code to send voltage signals to the motor.
    since the elevator has two motors geared together for more torque, we declare one as the main 
    which we will control directly, and one as the slave which just follows exactly what the main does

    !!!!!MAKE SURE THAT YOU DECLARE BRUSHLESS MOTORS AS BRUSHLESS AND BRUSHED AS BRUSHED!!!!
    I HAVE BLOWN UP MOTORS FROM MESSING THIS UP!!! */
  private static SparkMax elevatorMain = new SparkMax(CAN.ELEVATOR_MAIN, MotorType.kBrushless);
  private static SparkMax elevatorSlave = new SparkMax(CAN.ELEVATOR_SLAVE, MotorType.kBrushless);

  /*declare 'digital' motor controller object which will translate desired positions into direct
    signals to the motor controller using PID control. direct motor control (like in {CoralManipulatorSys})
    only allows motor velocity to be roughly controlled, while PID allows exact position and velocity control. 
    there are some great docs for this online that you should read before trying this yourself.
    highly recommend you read through the entirety of this: 
    https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/index.html
    
    getClosedLoopController() gets the built in controller setup inside the spark max. does not specify
    the physical encoder device yet.
    
    just need to read from the main motor since the slave will match */
  private static SparkClosedLoopController elevatorController = elevatorMain.getClosedLoopController();

  // declare configuration profiles for each motor. new setup as of 2025
  // https://docs.revrobotics.com/revlib/spark/configuring-a-spark
  // https://docs.revrobotics.com/revlib/configuring-devices
  private static SparkMaxConfig mainConfig = new SparkMaxConfig();
  private static SparkMaxConfig slaveConfig = new SparkMaxConfig();

  /*set up a motion profile to control the position signals sent to the motor controller.
    the pid controller tells the motor to move smoothly and accurately to a specific position. however, this 
    doesn't work great for moving the elevator across several feet of travel. one ex of why is there is no way to 
    directly set a maximum acceleration and velocity to this motion. with PID only control the maximum a and v reached 
    is a result of the tuning parameters and travel distance, meaning that if L1 to L4 reaches an unsafe velocity 
    and the controller is tuned to slow it down, L1 to L2 will be unnecessarily slow. instead a trapezoidal motion
    profile generates a profile of positions (setpoints) that the elevator moves to one at a time. trapezoidal means that if
    you graph position over time, the curve looks like a trapezoid--it accelerates constantly with MAX_A until it
    hits MAX_V, continues at that velocity until it is near the desired final target position, and then decelerates with
    MAX_A until it is stopped 
    https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html
    https://docs.wpilib.org/en/stable/docs/software/commandbased/profile-subsystems-commands.html 
    
    lowkey the best way to see if you need a motion profile vs direct pid control is try direct control since you'll
    need all the code from it for the profile anyway and then if it moves like garbage after some tuning use a profile */
  private final TrapezoidProfile motionProfile = 
    new TrapezoidProfile(new TrapezoidProfile.Constraints(ELEVATOR.PID.MAX_V, ELEVATOR.PID.MAX_A));
  // target is the desired final position
  private TrapezoidProfile.State target = new TrapezoidProfile.State();
  // setpoint is the intermediate steps along the trapezoidal curve
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  // unused limit switch intended to home the elevator. ultimately the elevator had no significant accumulated error
  // so homing was not necessary. may be an issue with future elevators so keep this in mind
  private DigitalInput homeLimit = new DigitalInput(0);

  public ElevatorSys() {
    // set up main config parameters, including the PID tuning parameters.
    // once you've read the wpilib general PID info this explains the implementation specifically
    // for revlib: https://docs.revrobotics.com/revlib/spark/closed-loop
    mainConfig
      .idleMode(IdleMode.kBrake)
      // !!!!MAKE SURE THAT INVERTING IS SET CORRECTLY FOR MOTORS GEARED TOGETHER LIKE THIS!!!!
      // check rotation direction with rev hardware client and motors in COAST MODE and make sure that
      // matches the inverting here BEFORE running any code or the motors will fight each other STALL OUT!!!
      .inverted(false)
      // another good idea with geared together motors is starting with a low amp limit so stalling doesn't
      // damage the motors if they're inverted incorrectly
      .smartCurrentLimit(ELEVATOR.AMP_LIMIT)
      .closedLoop
        .p(ELEVATOR.PID.P)
        .i(ELEVATOR.PID.I)
        .d(ELEVATOR.PID.D)
        .outputRange(ELEVATOR.PID.MIN, ELEVATOR.PID.MAX) 
        /*!!!! HERE IS WHERE THE ENCODER TYPE IS SET!! COMMON POINT OF ERROR IS SETTING THIS WRONG BECAUSE REV'S NAMING SUCKS!!!!
          kPrimaryEncoder reads the RELATIVE encoder built into the neo (explained at the top of this file around line 30).
          see {ManipulatorPitchSys} for PID with an absolute encoder */
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    // apply the config that was just defined to the main elevator sparkmax
    elevatorMain.configure(mainConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // largely same as above, key thing here is slave is set to follow main sparkmax. 
    // uses same PID signals as main so that doesn't need to be defined again
    slaveConfig
      .follow(elevatorMain)
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .smartCurrentLimit(ELEVATOR.AMP_LIMIT);
    elevatorSlave.configure(slaveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // inline command (rather than defining it in its own class file) to set height.
  // i believe this was just used for testing and isn't actually used; instead the {SetScoring} command
  // accesses the setProfile methods below
  public Command setHeight(double position) {
    return new SequentialCommandGroup(
      this.runOnce(() -> this.setProfile(position)),
      this.run(() -> this.followProfile()).until(() -> this.atHeight(position))
    );
  }

  // getter to check whether the elevator has reached the target position
  public boolean atHeight(double position) {
    if (Math.abs(getHeight() - position) < ELEVATOR.DEADBAND) {
      return true;
    } 
    return false;
  }

  // more unused home limit switch code. i'm pretty sure this is broken
  // public void homeEncoder() {
  //   if (!homeLimit.get()) {
  //    // elevatorMain.getEncoder().setPosition(0);
      
  //     //System.out.println(homeLimit.get());
  //   }
  // }

  // generate trapezoidal profile to target height (position) passed to method (run in {SetScoring} initialize)
  public void setProfile(double position) {
    // check if the target position is in the valid range of motion. VERY IMPORTANT TO PREVENT CRASHES!
    if (position <= ELEVATOR.MAX_COUNTS && position > 0) {
      // set profile target to be a point at the desired position with zero velocity--drive there and stop
      target = new TrapezoidProfile.State(position,0);
    }
    // send target position to dashboard
    SmartDashboard.putNumber("Elevator Setpoint",position);
  }
  
  // drives to target along profile; must be called repeatedly (runs in {SetScoring} execute)
  public void followProfile() {
    // gets setpoint object from motion profile
    setpoint = motionProfile.calculate(0.02, setpoint, target);
    // sets elevator pid controller to position from setpoint object with gravity feedforward
    // we pass a position value, specify it is a position value (not a velocity or voltage value) and give a 
    // feedforward value for the constant effort needed to fight gravity (explained in wpi advanced controls docs), 
    elevatorController.setReference(setpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ELEVATOR.PID.FF);
  }

  // getter to get ACTUAL current elevator position, NOT target
  public double getHeight() {
    return elevatorMain.getEncoder().getPosition();
  }

  // for the love of god just keep everything in the same unit instead of having to do stupid unit conversion methods
  // like this. they will introduce so much error when you forget them / do the math wrong. commented out for a reason
  // private double encoderToIn(double counts) {
  //   return ((counts + ELEVATOR.COUNT_OFFSET) / ELEVATOR.MAX_HEIGHT) * ELEVATOR.MAX_HEIGHT;
  // }

  @Override
  public void periodic() {
    // constantly update the dashboard with info about the elevator; crucial for tuning
    // to tune elevator, follow instructions from wpilib advanced controls docs, and plot target and real elevator
    // position together in advantagescope to easily analyze response time, error, and oscillations
    SmartDashboard.putNumber("Elevator Position", getHeight());
    SmartDashboard.putNumber("Slave Encoder", elevatorSlave.getEncoder().getPosition());
    SmartDashboard.putNumber("Elevator RPM", elevatorMain.getEncoder().getVelocity());
    SmartDashboard.putNumber("Elevator Amps", elevatorMain.getOutputCurrent() + elevatorSlave.getOutputCurrent());
    //homeEncoder();
    //SmartDashboard.putBoolean("Elevator Limit", homeLimit.get());
  }
}
