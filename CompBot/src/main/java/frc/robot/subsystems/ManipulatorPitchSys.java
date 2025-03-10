// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GLOBAL;
import frc.robot.Constants.MANIPULATOR;

public class ManipulatorPitchSys extends SubsystemBase {
  /** Creates a new ManipulatorPitch. */

  private static SparkMax manipulatorPitch = new SparkMax(Constants.CAN.MANIPULATOR_PITCH, MotorType.kBrushless);
  private static SparkClosedLoopController pitchController;

  //private static final AbsoluteEncoder manipulatorEncoder = manipulatorPitch.getAbsoluteEncoder();

  private static SparkMaxConfig pitchConfig = new SparkMaxConfig();

  GenericEntry pitchP;

  public ManipulatorPitchSys() {

    pitchController = manipulatorPitch.getClosedLoopController();

    pitchConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .p(MANIPULATOR.PID.P)
      .i(MANIPULATOR.PID.I)
      .d(MANIPULATOR.PID.D)
      .outputRange(MANIPULATOR.PID.MIN, MANIPULATOR.PID.MAX)
      .positionWrappingEnabled(true)
      .positionWrappingInputRange(0, 1);
    pitchConfig
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .smartCurrentLimit(MANIPULATOR.AMP_LIMIT);
    manipulatorPitch.configure(pitchConfig, ResetMode.kResetSafeParameters, null);

   
      pitchController.setReference(MANIPULATOR.HOME, ControlType.kPosition);
    

    //pitchP = Shuffleboard.getTab("Tuning").add("Pitch P Slider", 1).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 2)).getEntry();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Manipulator Encoder", getEncoder());
  }

  public void setPitch(double pitch) {

    pitchController.setReference(pitch + MANIPULATOR.MANIPULATOR_PIVOT_OFFSET, ControlType.kPosition);
  }

  public void setEncoder(double position) {
    //pitchController.setReference(position, ControlType.kPosition);
  }

  private double degreeToEncoder(double degrees) {
    return (degrees/360) + Constants.MANIPULATOR.MANIPULATOR_PIVOT_OFFSET;
    //add factor and constant for setpoint in testing
  }

  private double encoderToDegree(double counts) {
    return (counts + Constants.MANIPULATOR.MANIPULATOR_PIVOT_OFFSET) * 360;
  }

  public double getEncoder() {
    return manipulatorPitch.getAbsoluteEncoder().getPosition();
  }

  public double getPosition() {
    return encoderToDegree(manipulatorPitch.getAbsoluteEncoder().getPosition());
  }

  public void toHome() {
    setPitch(Constants.MANIPULATOR.HOME);
    if (GLOBAL.DEBUG_MODE) System.out.println("manip toHome");
  }

  public void toUnstick() {
    setPitch(Constants.MANIPULATOR.UNSTICK);
    if (GLOBAL.DEBUG_MODE) System.out.println("manip toUnstick");
  }

  public void toAGround() {
    setPitch(Constants.MANIPULATOR.AGROUND);
    if (GLOBAL.DEBUG_MODE) System.out.println("manip toAGround");
  }

  public void toAProcessor() {
    setPitch(Constants.MANIPULATOR.APROCESSOR);
    if (GLOBAL.DEBUG_MODE) System.out.println("manip toAProcessor");
  }

  public void toAReef() {
    setPitch(Constants.MANIPULATOR.AREEF);
    if (GLOBAL.DEBUG_MODE) System.out.println("manip toAReef");
  }

  public void toANet() {
    setPitch(Constants.MANIPULATOR.ANET);
    if (GLOBAL.DEBUG_MODE) System.out.println("manip toANet");
  }

  public void toCLow() {
    setPitch(Constants.MANIPULATOR.CLOW);
    if (GLOBAL.DEBUG_MODE) System.out.println("manip toCLow");
  }

  public void toCL1() {
    setPitch(Constants.MANIPULATOR.CL1);
    if (GLOBAL.DEBUG_MODE) System.out.println("manip toCL1");
  }

  public void toCHigh() {
    setPitch(Constants.MANIPULATOR.CHIGH);
    if (GLOBAL.DEBUG_MODE) System.out.println("manip toCHigh");
  }

  public boolean isHome() {
    if (getEncoder() <= MANIPULATOR.HOME + MANIPULATOR.DEADBAND) return true;
    return false;
  }

  public boolean isAtUnstick() {
    if (getEncoder() >= MANIPULATOR.UNSTICK - MANIPULATOR.DEADBAND && getEncoder() <= MANIPULATOR.UNSTICK + MANIPULATOR.DEADBAND) return true;
    return false;
  }

  public boolean isAtAGround() {
    if (getEncoder() >= MANIPULATOR.AGROUND - MANIPULATOR.DEADBAND && getEncoder() <= MANIPULATOR.AGROUND + MANIPULATOR.DEADBAND) return true;
    return false;
  }

  public boolean isAtAProcessor() {
    if (getEncoder() >= MANIPULATOR.APROCESSOR - MANIPULATOR.DEADBAND && getEncoder() <= MANIPULATOR.APROCESSOR + MANIPULATOR.DEADBAND) return true;
    return false;
  }

  public boolean isAtAReef() {
    if (getEncoder() >= MANIPULATOR.AREEF - MANIPULATOR.DEADBAND && getEncoder() <= MANIPULATOR.AREEF + MANIPULATOR.DEADBAND) return true;
    return false;
  }

  public boolean isAtANet() {
    if (getEncoder() >= MANIPULATOR.ANET - MANIPULATOR.DEADBAND && getEncoder() <= MANIPULATOR.ANET + MANIPULATOR.DEADBAND) return true;
    return false;
  }

  public boolean isAtCLow() {
    if (getEncoder() >= MANIPULATOR.CLOW - MANIPULATOR.DEADBAND && getEncoder() <= MANIPULATOR.CLOW + MANIPULATOR.DEADBAND) return true;
    return false;
  }

  public boolean isAtCHigh() {
    if (getEncoder() >= MANIPULATOR.CHIGH - MANIPULATOR.DEADBAND && getEncoder() <= MANIPULATOR.CHIGH + MANIPULATOR.DEADBAND) return true;
    return false;
  }
}
