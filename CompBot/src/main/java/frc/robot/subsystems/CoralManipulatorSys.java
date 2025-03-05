// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CORAL;
import frc.robot.Constants.DIO;

public class CoralManipulatorSys extends SubsystemBase {
  /** Creates a new CoralManipulator. */

  private static SparkMax coralManipulator = new SparkMax(Constants.CAN.CORAL_MANIPULATOR, MotorType.kBrushed);
  DigitalInput coralLimitSwitch = new DigitalInput(DIO.CORAL_LIMIT);

  public CoralManipulatorSys() {
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Coral Limit Switch", coralLimitSwitch.get());
    // This method will be called once per scheduler run
  }

  public void manipulateCoral(double speed) {
    coralManipulator.set(speed);
  }

  public void intakeCoral() {
    coralManipulator.set(CORAL.INTAKE_SPEED);
  }

  //for scoring L1-L3
  public void frontOuttakeCoral() {
    coralManipulator.set(CORAL.FRONT_OUTTAKE_SPEED);
  }

  //for scoring L4
  public void backOuttakeCoral() {
    coralManipulator.set(CORAL.BACK_OUTTAKE_SPEED);
  }

  public void stop() {
    coralManipulator.stopMotor();
  }
}
