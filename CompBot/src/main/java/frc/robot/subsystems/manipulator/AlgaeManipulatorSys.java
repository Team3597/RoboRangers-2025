// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import frc.robot.Constants.ALGAE;
import frc.robot.Constants.MANIPULATOR;

// this class has remnants of an attempt at PID velocity control to securely grip algae. however i was never
// able to get it working so all PID stuff will be left unexplained and this file should solely be an example
// of wiring digital inputs through sparkmaxes as limit switches

public class AlgaeManipulatorSys extends SubsystemBase {
  /** Creates a new AlgaeManipulator. */

  private static SparkMax algaeManipulator = new SparkMax(Constants.CAN.ALGAE_MANIPULATOR, MotorType.kBrushless);
  //https://docs.revrobotics.com/revlib/spark/closed-loop/closed-loop-control-getting-started
  private static SparkClosedLoopController algaeVelocityController = algaeManipulator.getClosedLoopController();

  private static SparkMaxConfig algaeConfig = new SparkMaxConfig();
  //Updated config docs: https://docs.revrobotics.com/revlib/configuring-devices

  // declare booleans for state of beambreaks
  public static boolean hasAlgae;
  public static boolean hasCoral;

  public AlgaeManipulatorSys() {
    algaeConfig.closedLoop
      .p(ALGAE.PID.P)
      .i(ALGAE.PID.I)
      .d(ALGAE.PID.D)
      .outputRange(ALGAE.PID.MIN, ALGAE.PID.MAX)
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      //includes feedforward due to constant control effort to rotate wheels
      .velocityFF(ALGAE.PID.FF);

    algaeConfig
      .inverted(true)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(MANIPULATOR.ALGAE_AMP_LIMIT);

    // disable limit switches stopping the motor, since our limit switches are actually unrelated coral
    // and algae beambreaks
    algaeConfig.limitSwitch
      .forwardLimitSwitchEnabled(false)
      .reverseLimitSwitchEnabled(false);

    algaeManipulator.configure(algaeConfig, null, null);

    algaeVelocityController.setReference(0, ControlType.kVelocity);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Algae Wheel Velocity", getEncoder());
    //System.out.println(algaeManipulator.getEncoder().getVelocity());
    SmartDashboard.putBoolean("Algae Beambreak", algaeManipulator.getForwardLimitSwitch().isPressed());
    SmartDashboard.putBoolean("Coral Beambreak", algaeManipulator.getReverseLimitSwitch().isPressed());

    // how to read the beambreaks. get limit switch object from sparkmax and call ispressed on it
    hasCoral = algaeManipulator.getReverseLimitSwitch().isPressed();
    hasAlgae = !algaeManipulator.getForwardLimitSwitch().isPressed();
  }

  public void holdAlgae() {
    algaeManipulator.set(-0.03);
    //runCoralCmd(1).andThen(Commands.waitSeconds(3)).andThen(Commands.runOnce(() -> coralManipulator.stopMotor())).schedule();
       
  }

  public void intakeAlgae() {
    //algaeVelocityController.setReference(MOTION.ALGAE_INTAKE_RPM, ControlType.kVelocity);
    algaeManipulator.set(-0.2);
  }

  public void outtakeAlgae() {
    //algaeVelocityController.setReference(MOTION.ALGAE_OUTTAKE_RPM, ControlType.kVelocity);
    algaeManipulator.set(1);
  }

  public void manipulateAlgae(double rpm) {
    algaeManipulator.set(rpm);
    //algaeVelocityController.setReference(rpm, ControlType.kVelocity);
  }

  public void stop() {
    algaeManipulator.stopMotor();
    //algaeVelocityController.setReference(0, ControlType.kVelocity);
  }

  public double getEncoder() {
    return algaeManipulator.getEncoder().getVelocity();
  }

  public double getVelocity() {
    return algaeManipulator.getEncoder().getVelocity();
  }
}
