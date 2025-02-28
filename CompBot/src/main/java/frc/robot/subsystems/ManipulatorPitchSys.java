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
      .p(Constants.PID.PITCH_P)
      .i(Constants.PID.PITCH_I)
      .d(Constants.PID.PITCH_D)
      .outputRange(Constants.PID.PITCH_MIN, Constants.PID.PITCH_MAX);
    pitchConfig
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .smartCurrentLimit(MANIPULATOR.AMP_LIMIT);
    manipulatorPitch.configure(pitchConfig, ResetMode.kResetSafeParameters, null);

    if (!GLOBAL.DISABLE_MANIPULATOR_PITCH) {
      pitchController.setReference(MANIPULATOR.HOME, ControlType.kPosition);
    }

    //pitchP = Shuffleboard.getTab("Tuning").add("Pitch P Slider", 1).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 2)).getEntry();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Manipulator Encoder", getEncoder());
  }

  public void updatePID() {

    Constants.PID.PITCH_P = pitchP.getDouble(0.5); //SmartDashboard.getNumber("Pitch P getnum", 0.5); 

    pitchConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .p(Constants.PID.PITCH_P)
      .i(Constants.PID.PITCH_I)
      .d(Constants.PID.PITCH_D)
      .outputRange(Constants.PID.PITCH_MIN, Constants.PID.PITCH_MAX);
    pitchConfig
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .smartCurrentLimit(MANIPULATOR.AMP_LIMIT);
    manipulatorPitch.configure(pitchConfig, null, null);


  }

  public void setPitch(double pitch) {
    if (!GLOBAL.DISABLE_MANIPULATOR_PITCH) {
      pitchController.setReference(degreeToEncoder(pitch), ControlType.kPosition);
      System.out.println("Setting pitch to " + pitch);
    }
  }

  public void setEncoder(double position) {
    if (!GLOBAL.DISABLE_MANIPULATOR_PITCH) {
      pitchController.setReference(position, ControlType.kPosition);
    }
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
    setEncoder(Constants.MANIPULATOR.HOME);
  }

  public void toUnstick() {
    setEncoder(Constants.MANIPULATOR.UNSTICK);
  }

  public void toAGround() {
    setEncoder(Constants.MANIPULATOR.AGROUND);
  }

  public void toAProcessor() {
    setEncoder(Constants.MANIPULATOR.APROCESSOR);
  }

  public void toAReef() {
    setEncoder(Constants.MANIPULATOR.AREEF);
  }

  public void toANet() {
    setEncoder(Constants.MANIPULATOR.ANET);
  }

  public void toCLow() {
    setEncoder(Constants.MANIPULATOR.CLOW);
  }

  public void toCHigh() {
    setEncoder(Constants.MANIPULATOR.CHIGH);
  }
}
