// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ManipulatorPitchSys extends SubsystemBase {
  /** Creates a new ManipulatorPitch. */

  private static SparkMax manipulatorPitch = new SparkMax(Constants.CAN.MANIPULATOR_PITCH, MotorType.kBrushless);
  private static SparkClosedLoopController pitchController;
  private static AbsoluteEncoder pitchEncoder;

  //private static final AbsoluteEncoder manipulatorEncoder = manipulatorPitch.getAbsoluteEncoder();

  private static SparkMaxConfig pitchConfig = new SparkMaxConfig();

  public ManipulatorPitchSys() {
    pitchController = manipulatorPitch.getClosedLoopController();
    pitchEncoder = manipulatorPitch.getAbsoluteEncoder();

    pitchConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .p(Constants.PID.PITCH_P)
      .i(Constants.PID.PITCH_I)
      .d(Constants.PID.PITCH_D)
      .outputRange(Constants.PID.PITCH_MIN, Constants.PID.PITCH_MAX);
    manipulatorPitch.configure(pitchConfig, null, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void setPitch(double pitch) {
    pitchController.setReference(degreeToEncoder(pitch), ControlType.kPosition);
  }

  private double degreeToEncoder(double degrees) {
    return (degrees/360)*Constants.MANIPULATOR.THROUGHBORE_CPR + Constants.MANIPULATOR.MANIPULATOR_PIVOT_OFFSET;
    //add factor and constant for setpoint in testing
  }

  private double encoderToDegree(double counts) {
    return ((counts + Constants.MANIPULATOR.MANIPULATOR_PIVOT_OFFSET)/Constants.MANIPULATOR.THROUGHBORE_CPR) * 360;
  }

  public double getEncoder() {
    return manipulatorPitch.getEncoder().getPosition();
  }

  public double getPosition() {
    return encoderToDegree(manipulatorPitch.getEncoder().getPosition());
  }

  public void toHome() {
    setPitch(Constants.MANIPULATOR.HOME);
  }

  public void toUnstick() {
    setPitch(Constants.MANIPULATOR.UNSTICK);
  }

  public void toAGround() {
    setPitch(Constants.MANIPULATOR.AGROUND);
  }

  public void toAProcessor() {
    setPitch(Constants.MANIPULATOR.APROCESSOR);
  }

  public void toAReef() {
    setPitch(Constants.MANIPULATOR.AREEF);
  }

  public void toANet() {
    setPitch(Constants.MANIPULATOR.ANET);
  }

  public void toCLow() {
    setPitch(Constants.MANIPULATOR.CLOW);
  }

  public void toCHigh() {
    setPitch(Constants.MANIPULATOR.CHIGH);
  }
}
