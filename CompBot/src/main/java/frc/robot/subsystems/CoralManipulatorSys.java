// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DIO;
import frc.robot.Constants.GLOBAL;
import frc.robot.Constants.MANIPULATOR;
import frc.robot.Constants.MOTION;

public class CoralManipulatorSys extends SubsystemBase {
  /** Creates a new CoralManipulator. */

  private static SparkMax coralManipulator = new SparkMax(Constants.CAN.CORAL_MANIPULATOR, MotorType.kBrushed);

  private static SparkMaxConfig coralConfig = new SparkMaxConfig();

  DigitalInput coralLimitSwitch = new DigitalInput(DIO.CORAL_LIMIT);

  public CoralManipulatorSys() {
    coralConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(MANIPULATOR.CORAL_AMP_LIMIT);
    coralManipulator.configure(coralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Coral Limit Switch", coralLimitSwitch.get());
    // This method will be called once per scheduler run
  }

  public void intakeCoral() {
    while (!coralLimitSwitch.get()) {
      coralManipulator.set(MOTION.CORAL_INTAKE_SPEED);
    }
    if (GLOBAL.DEBUG_MODE) {
      System.out.println("Intaking Coral");
    }
  }

  //for scoring L1-L3
  public void frontOuttakeCoral() {
    coralManipulator.set(MOTION.CORAL_FRONT_OUTTAKE_SPEED);
    if (GLOBAL.DEBUG_MODE) {
      System.out.println("frontOuttakeCoral");
    }
  }

  //for scoring L4
  public void backOuttakeCoral() {
    coralManipulator.set(MOTION.CORAL_BACK_OUTTAKE_SPEED);
    if (GLOBAL.DEBUG_MODE) {
      System.out.println("backOuttakeCoral");
    }
  }

  public void stop() {
    coralManipulator.stopMotor();;
  }
}
