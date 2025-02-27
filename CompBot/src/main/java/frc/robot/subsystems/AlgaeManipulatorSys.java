// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeManipulatorSys extends SubsystemBase {
  /** Creates a new AlgaeManipulator. */

  private static SparkMax algaeManipulator = new SparkMax(Constants.CAN.ALGAE_MANIPULATOR, MotorType.kBrushed);
  //https://docs.revrobotics.com/revlib/spark/closed-loop/closed-loop-control-getting-started
  private static SparkClosedLoopController algaeVelocityController = algaeManipulator.getClosedLoopController();

  private static SparkMaxConfig algaeConfig = new SparkMaxConfig();
  //Updated config docs: https://docs.revrobotics.com/revlib/configuring-devices

  public AlgaeManipulatorSys() {
    algaeConfig.closedLoop
      .p(Constants.PID.ALGAE_P)
      .i(Constants.PID.ALGAE_I)
      .d(Constants.PID.ALGAE_D)
      .outputRange(Constants.PID.ALGAE_MIN, Constants.PID.ALGAE_MAX)
      //includes feedforward due to constant control effort to rotate wheels
      .velocityFF(Constants.PID.ALGAE_FF);
    algaeManipulator.configure(algaeConfig, null, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeAlgae(double velocity) {
    algaeVelocityController.setReference(velocity, ControlType.kVelocity);
  }

  public void outtakeAlgae(double velocity) {
    algaeVelocityController.setReference(velocity, ControlType.kVelocity);
  }

  public double getEncoder() {
    return algaeManipulator.getEncoder().getPosition();
  }

  public double Velocity() {
    return algaeManipulator.getEncoder().getPosition();
  }
}
