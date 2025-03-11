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
      //includes feedforward due to gravity
      //.velocityFF(ELEVATOR.PID.FF)
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    // mainConfig.closedLoop.maxMotion
    // .maxVelocity(ELEVATOR.PID.MAX_V) //RPM by default
    // .maxAcceleration(ELEVATOR.PID.MAX_A) //RPM/S by default
    // .allowedClosedLoopError(1);
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
    SmartDashboard.putNumber("Elevator Encoder", GetElevatorEncoder());
    SmartDashboard.putNumber("Slave Encoder", elevatorSlave.getEncoder().getPosition());
  //  System.out.println(GetElevatorEncoder());
  //  System.out.println(elevatorSlave.getEncoder().getPosition());
    SmartDashboard.putNumber("Elevator RPM", elevatorMain.getEncoder().getVelocity());
    SmartDashboard.putNumber("Elevator Amps", elevatorMain.getOutputCurrent() + elevatorSlave.getOutputCurrent());
  }


  private void setHeight(double in) {

    // if (in < ELEVATOR.MAX_HEIGHT && in > 0) {
    //     elevatorController.setReference(inToEncoder(in), ControlType.kMAXMotionPositionControl,ClosedLoopSlot.kSlot0,ELEVATOR.PID.FF);
    // }
    target = new TrapezoidProfile.State(inToEncoder(in),0);
  }

  public void moveToHeight() {
    setpoint = motionProfile.calculate(0.02, setpoint, target);
    elevatorController.setReference(setpoint.position, ControlType.kPosition,ClosedLoopSlot.kSlot0,ELEVATOR.PID.FF);
  }

  private void setPosition(double count) {
    if (count < ELEVATOR.MAX_HEIGHT && count > 0) {
      elevatorController.setReference(count, ControlType.kPosition);

    }
  }

  private double inToEncoder(double in) {
    System.out.println ((in / ELEVATOR.MAX_HEIGHT) * ELEVATOR.MAX_COUNTS + ELEVATOR.COUNT_OFFSET);
    return (in / ELEVATOR.MAX_HEIGHT) * ELEVATOR.MAX_COUNTS + ELEVATOR.COUNT_OFFSET;
  }

  private double encoderToIn(double counts) {
    return ((counts + ELEVATOR.COUNT_OFFSET) / ELEVATOR.MAX_HEIGHT) * ELEVATOR.MAX_HEIGHT;
  }

  public double GetElevatorEncoder() {
    return encoderToIn(elevatorMain.getEncoder().getPosition());
  }

  public double GetElevatorPosition() {
    return encoderToIn(elevatorMain.getEncoder().getPosition());
  }

  public void toHome() {
    target = new TrapezoidProfile.State(ELEVATOR.HOME,0);
    //setHeight(Constants.ELEVATOR.HOME,false);
    //if (GLOBAL.DEBUG_MODE) System.out.println("elev toHome");
    SmartDashboard.putNumber("Setpoint",ELEVATOR.HOME);
  }

  public void toClear() {
    setHeight(Constants.ELEVATOR.CLEAR);
    if (GLOBAL.DEBUG_MODE) System.out.println("elev toClear");
  }

  public void toAL1() {
    setHeight(Constants.ELEVATOR.AL1);
    if (GLOBAL.DEBUG_MODE) System.out.println("elev toAL1");
  }

  public void toAL2() {
    setHeight(Constants.ELEVATOR.AL2);
    if (GLOBAL.DEBUG_MODE) System.out.println("elev toAL2");
  }

  public void toANet() {
    setHeight(Constants.ELEVATOR.ANET);
    if (GLOBAL.DEBUG_MODE) System.out.println("elev toANet");
  }

  public void toCL1() {
    setHeight(Constants.ELEVATOR.CL1);
    if (GLOBAL.DEBUG_MODE) System.out.println("elev toCL1");
    SmartDashboard.putNumber("Setpoint",ELEVATOR.CL1);
  }

  public void toCL2() {
    setHeight(Constants.ELEVATOR.CL2);
    if (GLOBAL.DEBUG_MODE) System.out.println("elev toCL2");
    SmartDashboard.putNumber("Setpoint",ELEVATOR.CL2);
  }

  public void toCL3() {
    setHeight(Constants.ELEVATOR.CL3);
    if (GLOBAL.DEBUG_MODE) System.out.println("elev toCL3");
    SmartDashboard.putNumber("Setpoint",ELEVATOR.CL3);
  }

  public void toCL4() {
    setHeight(Constants.ELEVATOR.CL4);
    if (GLOBAL.DEBUG_MODE) System.out.println("elev toCL4");
    SmartDashboard.putNumber("Setpoint",ELEVATOR.CL4);
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
