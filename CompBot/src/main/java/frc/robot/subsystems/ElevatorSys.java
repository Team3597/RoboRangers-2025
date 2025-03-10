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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.GLOBAL;

public class ElevatorSys extends SubsystemBase {
  /** Creates a new Elevator. */

  private static SparkMax elevatorMain = new SparkMax(Constants.CAN.ELEVATOR_MAIN, MotorType.kBrushless);
  private static SparkMax elevatorSlave = new SparkMax(Constants.CAN.ELEVATOR_SLAVE, MotorType.kBrushless);

  private static SparkClosedLoopController elevatorController = elevatorMain.getClosedLoopController();

  private static SparkMaxConfig mainConfig = new SparkMaxConfig();
  private static SparkMaxConfig slaveConfig = new SparkMaxConfig();

  private final TrapezoidProfile motionProfile = 
    new TrapezoidProfile(new TrapezoidProfile.Constraints(ELEVATOR.PID.MAX_V, ELEVATOR.PID.MAX_A));
  private TrapezoidProfile.State target = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  public ElevatorSys() {
    mainConfig.closedLoop
      .p(ELEVATOR.PID.P)
      .i(ELEVATOR.PID.I)
      .d(ELEVATOR.PID.D)
      .outputRange(ELEVATOR.PID.MIN, ELEVATOR.PID.MAX)
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    mainConfig
      .idleMode(IdleMode.kBrake)
      .inverted(false)
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
    SmartDashboard.putNumber("Elevator Position", GetElevatorPosition());
    SmartDashboard.putNumber("Slave Encoder", elevatorSlave.getEncoder().getPosition());
    SmartDashboard.putNumber("Elevator RPM", elevatorMain.getEncoder().getVelocity());
    SmartDashboard.putNumber("Elevator Amps", elevatorMain.getOutputCurrent() + elevatorSlave.getOutputCurrent());
  }

  //sets height target
  private void setHeight(double in) {
    if (in < ELEVATOR.MAX_HEIGHT && in > 0) {
      target = new TrapezoidProfile.State(in,0);
    }
    SmartDashboard.putNumber("Elevator Setpoint",in);
  }
  
  //drives to height target; must be called repeatedly
  public void moveToHeight() {
    //gets setpoint object from motion profile
    setpoint = motionProfile.calculate(0.02, setpoint, target);
    //sets elevator pid controller to position from setpoint object with gravity feedforward
    elevatorController.setReference(setpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ELEVATOR.PID.FF);
  }

  private double encoderToIn(double counts) {
    return ((counts + ELEVATOR.COUNT_OFFSET) / ELEVATOR.MAX_HEIGHT) * ELEVATOR.MAX_HEIGHT;
  }

  public double GetElevatorEncoder() {
    return elevatorMain.getEncoder().getPosition();
  }

  public double GetElevatorPosition() {
    return encoderToIn(elevatorMain.getEncoder().getPosition());
  }

  public void toHome() {
    setHeight(ELEVATOR.CLEAR);
    if (GLOBAL.DEBUG_MODE) System.out.println("elev toHome");
  }

  public void toClear() {
    setHeight(ELEVATOR.CLEAR);
    if (GLOBAL.DEBUG_MODE) System.out.println("elev toClear");
  }

  public void toAL1() {
    setHeight(ELEVATOR.AL1);
    if (GLOBAL.DEBUG_MODE) System.out.println("elev toAL1");
  }

  public void toAL2() {
    setHeight(ELEVATOR.AL2);
    if (GLOBAL.DEBUG_MODE) System.out.println("elev toAL2");
  }

  public void toANet() {
    setHeight(ELEVATOR.ANET);
    if (GLOBAL.DEBUG_MODE) System.out.println("elev toANet");
  }

  public void toCL1() {
    setHeight(ELEVATOR.CL1);
    if (GLOBAL.DEBUG_MODE) System.out.println("elev toCL1");
  }

  public void toCL2() {
    setHeight(ELEVATOR.CL2);
    if (GLOBAL.DEBUG_MODE) System.out.println("elev toCL2");
  }

  public void toCL3() {
    setHeight(ELEVATOR.CL3);
    if (GLOBAL.DEBUG_MODE) System.out.println("elev toCL3");
  }

  public void toCL4() {
    setHeight(ELEVATOR.CL4);
    if (GLOBAL.DEBUG_MODE) System.out.println("elev toCL4");
  }

  public boolean isHome() {
    if (GetElevatorPosition() <= ELEVATOR.HOME + ELEVATOR.DEADBAND) return true;
    return false;
  }
  
  public boolean isClear() {
    if (GetElevatorPosition() >= ELEVATOR.CLEAR - ELEVATOR.DEADBAND) {
     // System.out.println("IS CLEAR");
      return true;
    } else {
     // System.out.println("NOT CLEAR");
    return false;
    }
  }

  public boolean isAtAL1() {
    if (GetElevatorPosition() >= ELEVATOR.AL1 - ELEVATOR.DEADBAND && GetElevatorPosition() <= ELEVATOR.AL1 + ELEVATOR.DEADBAND) return true;
    return false;
  }

  public boolean isAtAL2() {
    if (GetElevatorPosition() >= ELEVATOR.AL2 - ELEVATOR.DEADBAND && GetElevatorPosition() <= ELEVATOR.AL2 + ELEVATOR.DEADBAND) return true;
    return false;
  }

  public boolean isAtANet() {
    if (GetElevatorPosition() >= ELEVATOR.ANET - ELEVATOR.DEADBAND && GetElevatorPosition() <= ELEVATOR.ANET + ELEVATOR.DEADBAND) return true;
    return false;
  }

  public boolean isAtCL1() {
    if (GetElevatorPosition() >= ELEVATOR.CL1 - ELEVATOR.DEADBAND && GetElevatorPosition() <= ELEVATOR.CL1 + ELEVATOR.DEADBAND) return true;
    return false;
  }

  public boolean isAtCL2() {
    if (GetElevatorPosition() >= ELEVATOR.CL2 - ELEVATOR.DEADBAND && GetElevatorPosition() <= ELEVATOR.CL2 + ELEVATOR.DEADBAND) return true;
    return false;
  }

  public boolean isAtCL3() {
    if (GetElevatorPosition() >= ELEVATOR.CL3 - ELEVATOR.DEADBAND && GetElevatorPosition() <= ELEVATOR.CL3 + ELEVATOR.DEADBAND) return true;
    return false;
  }

  public boolean isAtCL4() {
    if (GetElevatorPosition() >= ELEVATOR.CL4 - ELEVATOR.DEADBAND && GetElevatorPosition() <= ELEVATOR.CL4 + ELEVATOR.DEADBAND) return true;
    return false;
  }
}
