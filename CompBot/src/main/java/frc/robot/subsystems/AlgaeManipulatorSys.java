// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GLOBAL;
import frc.robot.Constants.MANIPULATOR;
import frc.robot.Constants.MOTION;

public class AlgaeManipulatorSys extends SubsystemBase {
  /** Creates a new AlgaeManipulator. */

  private static SparkMax algaeManipulator = new SparkMax(Constants.CAN.ALGAE_MANIPULATOR, MotorType.kBrushless);
  //https://docs.revrobotics.com/revlib/spark/closed-loop/closed-loop-control-getting-started
  private static SparkClosedLoopController algaeVelocityController = algaeManipulator.getClosedLoopController();

  private static SparkMaxConfig algaeConfig = new SparkMaxConfig();
  //Updated config docs: https://docs.revrobotics.com/revlib/configuring-devices

  //private static Ultrasonic algaeDistance = new Ultrasonic(null, null)

  public AlgaeManipulatorSys() {
    algaeConfig.closedLoop
      .p(Constants.PID.ALGAE_P)
      .i(Constants.PID.ALGAE_I)
      .d(Constants.PID.ALGAE_D)
      .outputRange(Constants.PID.ALGAE_MIN, Constants.PID.ALGAE_MAX)
      //includes feedforward due to constant control effort to rotate wheels
      .velocityFF(Constants.PID.ALGAE_FF);
      //.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    algaeConfig
      .inverted(true)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(MANIPULATOR.ALGAE_AMP_LIMIT);
    algaeManipulator.configure(algaeConfig, null, null);

    algaeVelocityController.setReference(0, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Algae Wheel Velocity", getEncoder());
    //System.out.println(algaeManipulator.getEncoder().getVelocity());
  }

  public void intakeAlgae() {
    algaeVelocityController.setReference(MOTION.ALGAE_INTAKE_RPM, ControlType.kVelocity);
    //algaeManipulator.set(MOTION.ALGAE_INTAKE_SPEED);
    if (GLOBAL.DEBUG_MODE) {
      System.out.println("intakeAlgae");
    }
  }

  public void outtakeAlgae() {
    algaeVelocityController.setReference(MOTION.ALGAE_OUTTAKE_RPM, ControlType.kVelocity);
    //algaeManipulator.set(MOTION.ALGAE_OUTTAKE_SPEED);
    if (GLOBAL.DEBUG_MODE) {
      System.out.println("outtakeAlgae");
    }
  }

  public void stop() {
    algaeVelocityController.setReference(0, ControlType.kVelocity);
    algaeManipulator.stopMotor();
  }

  public void hold() {
    algaeManipulator.set(MOTION.ALGAE_HOLD_SPEED);
  }

  public double getEncoder() {
    return algaeManipulator.getEncoder().getVelocity();
  }

  public double getVelocity() {
    return algaeManipulator.getEncoder().getVelocity();
  }
}
