// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CAN;
import frc.robot.Constants.ELEVATOR;

public class ElevatorSys extends SubsystemBase {

  private static SparkMax elevatorMain = new SparkMax(CAN.ELEVATOR_MAIN, MotorType.kBrushless);
  private static SparkMax elevatorSlave = new SparkMax(CAN.ELEVATOR_SLAVE, MotorType.kBrushless);

  private static SparkClosedLoopController elevatorController = elevatorMain.getClosedLoopController();

  private static SparkMaxConfig mainConfig = new SparkMaxConfig();
  private static SparkMaxConfig slaveConfig = new SparkMaxConfig();

  private final TrapezoidProfile motionProfile = 
    new TrapezoidProfile(new TrapezoidProfile.Constraints(ELEVATOR.PID.MAX_V, ELEVATOR.PID.MAX_A));
  private TrapezoidProfile.State target = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  public ElevatorSys() {
    mainConfig
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .smartCurrentLimit(ELEVATOR.AMP_LIMIT)
      .closedLoop
        .p(ELEVATOR.PID.P)
        .i(ELEVATOR.PID.I)
        .d(ELEVATOR.PID.D)
        .outputRange(ELEVATOR.PID.MIN, ELEVATOR.PID.MAX)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    elevatorMain.configure(mainConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    slaveConfig
      .follow(elevatorMain)
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .smartCurrentLimit(ELEVATOR.AMP_LIMIT);
    elevatorSlave.configure(slaveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command setHeight(double position) {
    return new SequentialCommandGroup(
      this.runOnce(() -> this.setProfile(position)),
      this.run(() -> this.followProfile()).until(() -> this.atHeight(position))
    );
  }

  public boolean atHeight(double position) {
    if (Math.abs(getHeight() - position) < ELEVATOR.DEADBAND) {
      return true;
    } 
    return false;
  }

  //create profile to target height
  public void setProfile(double position) {
    if (position < ELEVATOR.MAX_HEIGHT && position > 0) {
      target = new TrapezoidProfile.State(position,0);
    }
    SmartDashboard.putNumber("Elevator Setpoint",position);
  }
  
  //drives to target along profile; must be called repeatedly
  public void followProfile() {
    //gets setpoint object from motion profile
    setpoint = motionProfile.calculate(0.02, setpoint, target);
    //sets elevator pid controller to position from setpoint object with gravity feedforward
    elevatorController.setReference(setpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ELEVATOR.PID.FF);
  }

  public double getHeight() {
    return elevatorMain.getEncoder().getPosition();
  }

  // private double encoderToIn(double counts) {
  //   return ((counts + ELEVATOR.COUNT_OFFSET) / ELEVATOR.MAX_HEIGHT) * ELEVATOR.MAX_HEIGHT;
  // }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Position", getHeight());
    SmartDashboard.putNumber("Slave Encoder", elevatorSlave.getEncoder().getPosition());
    SmartDashboard.putNumber("Elevator RPM", elevatorMain.getEncoder().getVelocity());
    SmartDashboard.putNumber("Elevator Amps", elevatorMain.getOutputCurrent() + elevatorSlave.getOutputCurrent());
  }
}
