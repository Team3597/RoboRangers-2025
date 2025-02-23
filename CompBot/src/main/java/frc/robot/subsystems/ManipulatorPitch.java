// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ManipulatorPitch extends SubsystemBase {
  /** Creates a new ManipulatorPitch. */

  private static SparkMax manipulatorPitch = new SparkMax(Constants.CAN.MANIPULATOR_PITCH, MotorType.kBrushed);
  private static SparkClosedLoopController pitchController = manipulatorPitch.getClosedLoopController();

  private static SparkMaxConfig pitchConfig = new SparkMaxConfig();

  public ManipulatorPitch() {
    pitchConfig.closedLoop
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

  public void setPitch(double pitch) {
    pitchController.setReference(pitch, ControlType.kPosition);
  }


}
