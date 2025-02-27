// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  private static SparkMax elevatorMain = new SparkMax(Constants.CAN.ELEVATOR_MAIN, MotorType.kBrushed);
  private static SparkMax elevatorSlave = new SparkMax(Constants.CAN.ELEVATOR_SLAVE, MotorType.kBrushed);

  private static SparkClosedLoopController algaeVelocityController = algaeManipulator.getClosedLoopController();

  private static SparkMaxConfig algaeConfig = new SparkMaxConfig();

  public Elevator() {
    algaeConfig.closedLoop
    .p(Constants.PID.ALGAE_P)
    .i(Constants.PID.ALGAE_I)
    .d(Constants.PID.ALGAE_D)
    .outputRange(Constants.PID.ALGAE_MIN, Constants.PID.ALGAE_MAX)
    //includes feedforward due to gravity
    .velocityFF(Constants.PID.ALGAE_FF);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
