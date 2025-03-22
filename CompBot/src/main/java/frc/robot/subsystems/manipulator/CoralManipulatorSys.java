// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.CORAL;
import frc.robot.Constants.DIO;
import frc.robot.Constants.MANIPULATOR;
import frc.robot.commands.StepCoral;
import frc.robot.subsystems.StateSys;

public class CoralManipulatorSys extends SubsystemBase {
  /** Creates a new CoralManipulator. */

  private static SparkMax coralManipulator = new SparkMax(Constants.CAN.CORAL_MANIPULATOR, MotorType.kBrushed);

  private static SparkMaxConfig coralConfig = new SparkMaxConfig();

  private boolean hadCoral;

  private Command stepCoralForward;

  //DigitalInput coralLimitSwitch = new DigitalInput(DIO.CORAL_LIMIT);

  public CoralManipulatorSys() {
    coralConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(MANIPULATOR.CORAL_AMP_LIMIT);
    coralManipulator.configure(coralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

   // stepCoralForward = Commands.startEnd(() -> coralManipulator.set(CORAL.INTAKE_SPEED), () -> coralManipulator.stopMotor());
  }

  @Override
  public void periodic() {
    //SmartDashboard.putBoolean("Coral Limit Switch", coralLimitSwitch.get());
    // This method will be called once per scheduler run
  }

  public Command runCoralCmd(double speed) {
    return new Command() {
      public void initialize() {
        coralManipulator.set(speed);
        System.out.println("kys");
      }
    };
  }

  public void stepCoralCmd() {
   //Commands.sequence(Commands.startEnd(() -> coralManipulator.set(CORAL.INTAKE_SPEED), () -> coralManipulator.stopMotor()).withTimeout(3));
   System.out.println("stepCoral"); 
   Commands.sequence(
      Commands.runOnce(() -> coralManipulator.set(1)),
      Commands.waitSeconds(0.0),
      Commands.runOnce(() -> coralManipulator.stopMotor()),
      Commands.runOnce(() -> System.out.println("end"))
    ).schedule();
       //runCoralCmd(1).andThen(Commands.waitSeconds(3)).andThen(Commands.runOnce(() -> coralManipulator.stopMotor())).schedule();
       
  }

  public void manipulateCoral(double speed) {
    coralManipulator.set(speed);
  }

  public void intakeCoral() {
   if (!AlgaeManipulatorSys.hasCoral) {
      coralManipulator.set(CORAL.INTAKE_SPEED);
   } else {
      coralManipulator.stopMotor();
   }

  }



  //for scoring L1-L3
  public void frontOuttakeCoral() {

    coralManipulator.set(CORAL.FRONT_OUTTAKE_SPEED);

  }

  //for scoring L4
  public void backOuttakeCoral() {

    coralManipulator.set(-CORAL.BACK_OUTTAKE_SPEED);

  }

  public void stop() {
    coralManipulator.stopMotor();
  }
}
