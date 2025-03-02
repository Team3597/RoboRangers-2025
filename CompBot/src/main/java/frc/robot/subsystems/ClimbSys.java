// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
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
import frc.robot.Constants.PID;

public class ClimbSys extends SubsystemBase {
  /** Creates a new ClimbSys. */

  private static SparkMax climbPitch = new SparkMax(CAN.CLIMB, MotorType.kBrushless);
  private static SparkClosedLoopController pitchController;

  private static SparkMaxConfig pitchConfig = new SparkMaxConfig();

  public ClimbSys() {
    pitchController = climbPitch.getClosedLoopController();

    pitchConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .p(PID.CLIMB_P_POS)
      .i(PID.CLIMB_I_POS)
      .d(PID.CLIMB_D_POS)
      .outputRange(PID.CLIMB_MIN_POS, PID.CLIMB_MAX_POS)
      
      .p(PID.CLIMB_P_CLIMB, ClosedLoopSlot.kSlot1)
      .i(PID.CLIMB_I_CLIMB, ClosedLoopSlot.kSlot1)
      .d(PID.CLIMB_D_CLIMB, ClosedLoopSlot.kSlot1)
      .outputRange(PID.CLIMB_MIN_CLIMB, PID.CLIMB_MAX_CLIMB);

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
    pitchController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
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
    setEncoder(CLIMB.HOME);
  }

  public void toReady() {
    setEncoder(CLIMB.READY);
  }

  public void toLatched() {
    pitchController.setReference(CLIMB.LATCHED, ControlType.kPosition, ClosedLoopSlot.kSlot1);
  }
}
