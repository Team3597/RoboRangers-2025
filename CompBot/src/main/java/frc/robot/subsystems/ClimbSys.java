// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.CLIMB;

public class ClimbSys extends SubsystemBase {
  /** Creates a new ClimbSys. */

  private static SparkMax climbPitch = new SparkMax(CAN.CLIMB, MotorType.kBrushless);
  private static SparkClosedLoopController pitchController;

  private static SparkMaxConfig pitchConfig = new SparkMaxConfig();

  public ClimbSys() {
    pitchController = climbPitch.getClosedLoopController();

    pitchConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .p(Constants.PID.CLIMB_P)
      .i(Constants.PID.CLIMB_I)
      .d(Constants.PID.CLIMB_D)
      .outputRange(Constants.PID.CLIMB_MIN, Constants.PID.CLIMB_MAX);
    pitchConfig
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .smartCurrentLimit(CLIMB.AMP_LIMIT);
    climbPitch.configure(pitchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPitch(double pitch) {
    pitchController.setReference(degreeToEncoder(pitch), ControlType.kPosition);
  }

  public void setEncoder(double position) {
    pitchController.setReference(position, ControlType.kPosition);
  }

  private double degreeToEncoder(double degrees) {
    return (degrees/360) + Constants.MANIPULATOR.MANIPULATOR_PIVOT_OFFSET;
    //add factor and constant for setpoint in testing
  }

  private double encoderToDegree(double counts) {
    return ((counts + Constants.MANIPULATOR.MANIPULATOR_PIVOT_OFFSET)/Constants.MANIPULATOR.THROUGHBORE_CPR) * 360;
  }

  public double getEncoder() {
    return climbPitch.getAbsoluteEncoder().getPosition();
  }

  public double getPosition() {
    return encoderToDegree(climbPitch.getAbsoluteEncoder().getPosition());
  }

  public void toHome() {
    setPitch(CLIMB.HOME);
  }

  public void toReady() {
    setPitch(CLIMB.READY);
  }

  public void toLatched() {
    setPitch(degreeToEncoder(CLIMB.LATCHED));
  }
}
