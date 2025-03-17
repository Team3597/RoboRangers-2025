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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.MANIPULATOR;

public class ManipulatorPitchSys extends SubsystemBase {

  private static SparkMax manipulatorPitch = new SparkMax(CAN.MANIPULATOR_PITCH, MotorType.kBrushless);
  private static SparkClosedLoopController pitchController;
  private static SparkMaxConfig pitchConfig = new SparkMaxConfig();

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
  }

  public Command setPitch(double pitch) {
    return 
      this.run(() -> this.setReference(pitch)).until(() -> this.atPitch(pitch))
    ;
  }

  public boolean atPitch(double pitch) {
    if (Math.abs(getPitch() - pitch) < MANIPULATOR.DEADBAND) {
      return true;
    } 
    return false;
  }


  public void setReference(double pitch) {
    pitchController.setReference(pitch + MANIPULATOR.MANIPULATOR_PIVOT_OFFSET, ControlType.kPosition);
  }

  public double getPitch() {
    return manipulatorPitch.getAbsoluteEncoder().getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Manipulator Encoder", getPitch());
  }

}
