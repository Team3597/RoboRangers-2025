// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSys extends SubsystemBase {
  /** Creates a new Elevator. */

  private static SparkMax elevatorMain = new SparkMax(Constants.CAN.ELEVATOR_MAIN, MotorType.kBrushed);
  private static SparkMax elevatorSlave = new SparkMax(Constants.CAN.ELEVATOR_SLAVE, MotorType.kBrushed);

  private static SparkClosedLoopController elevatorController = elevatorMain.getClosedLoopController();

  private static SparkMaxConfig mainConfig = new SparkMaxConfig();
  private static SparkMaxConfig slaveConfig = new SparkMaxConfig();


  public ElevatorSys() {
    mainConfig.closedLoop
      .p(Constants.PID.ELEVATOR_P)
      .i(Constants.PID.ELEVATOR_I)
      .d(Constants.PID.ELEVATOR_D)
      .outputRange(Constants.PID.ELEVATOR_MIN, Constants.PID.ELEVATOR_MAX)
      //includes feedforward due to gravity
      .velocityFF(Constants.PID.ELEVATOR_FF);
    mainConfig
      .idleMode(IdleMode.kBrake)
      .inverted(true)
      .smartCurrentLimit(Constants.ELEVATOR.AMP_LIMIT);
    elevatorMain.configure(mainConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    slaveConfig
      .follow(elevatorMain)
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .smartCurrentLimit(Constants.ELEVATOR.AMP_LIMIT);
    elevatorSlave.configure(slaveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //toHome();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Encoder", GetElevatorEncoder());
    SmartDashboard.putNumber("Slave Encoder", elevatorSlave.getEncoder().getPosition());
  }

  private void setHeight(double in) {
    if (in < Constants.ELEVATOR.ELEVATOR_MAX_HEIGHT && in > 0) {
      elevatorController.setReference(-inToEncoder(in), ControlType.kPosition);
    }
  }

  private void setPosition(double count) {
    if (count < Constants.ELEVATOR.ELEVATOR_MAX_HEIGHT && count > 0) {
      elevatorController.setReference(-count, ControlType.kPosition);
    }
  }

  private double inToEncoder(double in) {
    return (in / Constants.ELEVATOR.ELEVATOR_MAX_HEIGHT) * Constants.ELEVATOR.ELEVATOR_MAX_COUNTS + Constants.ELEVATOR.ELEVATOR_COUNT_OFFSET;
  }

  private double encoderToIn(double counts) {
    return ((counts + Constants.ELEVATOR.ELEVATOR_COUNT_OFFSET) / Constants.ELEVATOR.ELEVATOR_MAX_HEIGHT) * Constants.ELEVATOR.ELEVATOR_MAX_HEIGHT;
  }

  public double GetElevatorEncoder() {
    return elevatorMain.getEncoder().getPosition();
  }

  public double GetElevatorPosition() {
    return encoderToIn(elevatorMain.getEncoder().getPosition());
  }

  public void toHome() {
    setHeight(Constants.ELEVATOR.HOME);
  }

  public void toClear() {
    setHeight(Constants.ELEVATOR.CLEAR);
  }

  public void toAProcessor() {
    setHeight(Constants.ELEVATOR.APROCESSOR);
  }

  public void toAL1() {
    setHeight(Constants.ELEVATOR.AL1);
  }

  public void toAL2() {
    setHeight(Constants.ELEVATOR.AL2);
  }

  public void toANet() {
    setHeight(Constants.ELEVATOR.ANET);
  }

  public void toCL1() {
    setHeight(Constants.ELEVATOR.CL1);
  }

  public void toCL2() {
    setHeight(Constants.ELEVATOR.CL2);
  }

  public void toCL3() {
    setHeight(Constants.ELEVATOR.CL3);
  }

  public void toCL4() {
    setHeight(Constants.ELEVATOR.CL4);
  }
}
