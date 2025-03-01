// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ManipulateObject;
import frc.robot.commands.UpdatePID;
import frc.robot.commands.setManipulatorPitch;
import frc.robot.commands.algae.AManipulate;
import frc.robot.commands.algae.ToAGround;
import frc.robot.commands.algae.ToAL1;
import frc.robot.commands.algae.ToAL2;
import frc.robot.commands.algae.ToANet;
import frc.robot.commands.algae.ToAProcessor;
import frc.robot.commands.climb.ToClimbHome;
import frc.robot.commands.climb.ToClimbLatched;
import frc.robot.commands.climb.ToClimbReady;
import frc.robot.commands.coral.CManipulate;
import frc.robot.commands.coral.ToCL1;
import frc.robot.commands.coral.ToCL2;
import frc.robot.commands.coral.ToCL3;
import frc.robot.commands.coral.ToCL4;
import frc.robot.commands.coral.ToHome;
import frc.robot.subsystems.AlgaeManipulatorSys;
import frc.robot.subsystems.ClimbSys;
import frc.robot.subsystems.CoralManipulatorSys;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ExampleSys;
import frc.robot.subsystems.ManipulatorPitchSys;
import frc.robot.subsystems.StateMonitorSys;
import frc.robot.subsystems.StateMonitorSys.ClimbState;
import frc.robot.subsystems.VisionSys;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.GLOBAL;
import frc.robot.Constants.MOTION;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = 
    new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                        "swerve"));

  private static final ExampleSys m_exampleSubsystem = new ExampleSys(); 

  private static final AlgaeManipulatorSys m_algaeManipulatorSys = new AlgaeManipulatorSys();
  private static final CoralManipulatorSys m_coralManipulatorSys = new CoralManipulatorSys();
  private static final ElevatorSys m_elevatorSys = new ElevatorSys();
  private static final ManipulatorPitchSys m_manpulatorPitchSys = new ManipulatorPitchSys();
  private static final ClimbSys m_climbSys = new ClimbSys();
  private static final VisionSys m_visionSys = new VisionSys();
  private static final StateMonitorSys m_stateMonitorSys = new StateMonitorSys();

  //private static DigitalInput m_coralLimit = new DigitalInput(Constants.DIO.CORAL_LIMIT);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_controlController = new CommandXboxController(Constants.OPERATOR.CONTROL_CONTROLLER_PORT);
  private final CommandXboxController m_driver2 = new CommandXboxController(Constants.OPERATOR.DRIVE_CONTROLLER_PORT_2);

  private final Joystick driveJoystick = new Joystick(Constants.OPERATOR.DRIVE_CONTROLLER_PORT);

    /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = 
    SwerveInputStream.of(drivebase.getSwerveDrive(),
                          () -> m_driver2.getLeftY() * 1, //LY, 1
                          () -> m_driver2.getLeftX() * 1) //LX, 0
                      .withControllerRotationAxis(m_driver2::getRightX) //Rotate 2
                      .deadband(OperatorConstants.DEADBAND)
                      .scaleTranslation(0.8)
                      .scaleRotation(0-0.4)
                      .allianceRelativeControl(false);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  // SwerveInputStream driveDirectAngle = 
  //   driveAngularVelocity.copy().withControllerHeadingAxis(m_controlController::getRightX,
  //                                                         m_controlController::getRightY)
  //                         .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = 
    driveAngularVelocity.copy().robotRelative(true)
                          .allianceRelativeControl(false);

  // SwerveInputStream driveAngularVelocityKeyboard 
  //   = SwerveInputStream.of(drivebase.getSwerveDrive(),
  //                           () -> -m_controlController.getLeftY(),
  //                           () -> -m_controlController.getLeftX())
  //                       .withControllerRotationAxis(() -> m_controlController.getRawAxis(
  //                           2))
  //                       .deadband(OperatorConstants.DEADBAND)
  //                       .scaleTranslation(0.8)
  //                       .allianceRelativeControl(true);
  // // Derive the heading axis with math!
  // SwerveInputStream driveDirectAngleKeyboard
  //   = driveAngularVelocityKeyboard.copy()
  //                                 .withControllerHeadingAxis(() ->
  //                                                               Math.sin(
  //                                                                   m_controlController.getRawAxis(
  //                                                                       2) *
  //                                                                   Math.PI) *
  //                                                               (Math.PI *
  //                                                                 2),
  //                                                           () ->
  //                                                               Math.cos(
  //                                                                   m_controlController.getRawAxis(
  //                                                                       2) *
  //                                                                   Math.PI) *
  //                                                               (Math.PI *
  //                                                                 2))
  //                                 .headingWhile(true);

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    
    //m_controlController.button(7).whileTrue(new IntakeC(m_coralManipulatorSys));

    
    //if (StateMonitorSys.climbState != ClimbState.LATCHED) {
      // coral controls
      //m_controlController.pov(0).onTrue(new ToCL4(m_elevatorSys, m_manpulatorPitchSys)); // up
      m_controlController.pov(90).onTrue(new ToCL3(m_elevatorSys, m_manpulatorPitchSys)); // right
      m_controlController.pov(180).onTrue(new ToCL1(m_elevatorSys, m_manpulatorPitchSys)); // down
      m_controlController.pov(270).onTrue(new ToCL2(m_elevatorSys, m_manpulatorPitchSys)); // left
      m_controlController.button(9).onTrue(new ToHome(m_elevatorSys, m_manpulatorPitchSys)); // left stick button


      m_controlController.leftBumper().whileTrue(new CManipulate(m_coralManipulatorSys, MOTION.CORAL_INTAKE_SPEED)); // left bumper
      m_controlController.button(7).whileTrue(new CManipulate(m_coralManipulatorSys, MOTION.CORAL_FRONT_OUTTAKE_SPEED));
      m_controlController.button(11).whileTrue(new CManipulate(m_coralManipulatorSys, -MOTION.CORAL_BACK_OUTTAKE_SPEED));//left stick

      m_controlController.rightBumper().whileTrue(new AManipulate(m_algaeManipulatorSys, MOTION.ALGAE_INTAKE_RPM)); // left bumper
      m_controlController.button(8).whileTrue(new AManipulate(m_algaeManipulatorSys, -MOTION.ALGAE_OUTTAKE_RPM));
     
      m_algaeManipulatorSys.setDefaultCommand(new AManipulate(m_algaeManipulatorSys, 0));

      // algae controls
    //  m_controlController.button(4).onTrue(new ToANet(m_elevatorSys, m_manpulatorPitchSys)); // y
      m_controlController.button(1).onTrue(new ToAL1(m_elevatorSys, m_manpulatorPitchSys)); // x
      m_controlController.button(3).onTrue(new ToAL2(m_elevatorSys, m_manpulatorPitchSys)); // b
      m_controlController.button(2).onTrue(new ToAProcessor(m_elevatorSys, m_manpulatorPitchSys)); // a
      m_controlController.button(10).onTrue(new ToAGround(m_elevatorSys, m_manpulatorPitchSys)); // start


      // climb controls
    //   if (StateMonitorSys.climbState == ClimbState.READY) { // back (toggles between climb home and ready)
    //     m_controlController.back().onTrue(new ToClimbHome(m_climbSys));
    //   } else if (StateMonitorSys.climbState == ClimbState.HOME) {
    //     m_controlController.back().onTrue(new ToClimbReady(m_climbSys));
    //   }
    //   // missing method to read center logitech button
    //   // m_controlController.centerLogitechButton().onTrue(new ToClimbLatched(m_climbSys)); // center logitech button
    // } else {
    //   m_controlController.rightStick().onTrue(new ToClimbHome(m_climbSys)); // Unlocks controls and resets climb if accidentally pressed (temporarily right stick button)

    m_driver2.button(0).onTrue(new ToClimbHome(m_climbSys)); //A
    m_driver2.button(1).onTrue(new ToClimbReady(m_climbSys)); //B
    m_driver2.button(4).onTrue(new ToClimbLatched(m_climbSys)); //Y

      // Trigger joystick3 = new JoystickButton(driveJoystick, 3);
      // joystick3.onTrue(new ToClimbHome(m_climbSys));

      // Trigger joystick5 = new JoystickButton(driveJoystick, 5);
      // joystick5.onTrue(new ToClimbReady(m_climbSys));

      // Trigger joystick6 = new JoystickButton(driveJoystick, 6);
      // joystick6.onTrue(new ToClimbLatched(m_climbSys));
   // }
    





    // POVButton upPov = new POVButton(m_controlController.getHID(), 0);
    // POVButton leftPov = new POVButton(m_controlController.getHID(), 270);
    // POVButton rightPov = new POVButton(m_controlController.getHID(), 90);
    // POVButton downPov = new POVButton(m_controlController.getHID(), 180);


    // Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    // Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    // Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    // Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    // Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      // drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      m_controlController.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      m_controlController.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveRobotOrientedAngularVelocity); // Overrides drive command above!

      m_controlController.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      m_controlController.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      m_controlController.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      m_controlController.back().whileTrue(drivebase.centerModulesCommand());
      m_controlController.leftBumper().onTrue(Commands.none());
      m_controlController.rightBumper().onTrue(Commands.none());
    } else
    {
      // m_controlController.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      // m_controlController.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      // m_controlController.b().whileTrue(
      //     drivebase.driveToPose(
      //         new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
      //                         );
      m_controlController.start().whileTrue(Commands.none());
      m_controlController.back().whileTrue(Commands.none());
      m_controlController.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      m_controlController.rightBumper().onTrue(Commands.none());


    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
  //  return new SequentialCommandGroup( 
     return drivebase.driveToDistanceCommand(5, 1.5).withTimeout(2);
    //   new ToCL3(m_elevatorSys, m_manpulatorPitchSys),
    //   drivebase.driveToDistanceCommand(0.2, -0.5).withTimeout(2),
    //   new CManipulate(m_coralManipulatorSys, MOTION.CORAL_FRONT_OUTTAKE_SPEED)
    // );
    
    //Autos.exampleAuto(m_exampleSubsystem);
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
