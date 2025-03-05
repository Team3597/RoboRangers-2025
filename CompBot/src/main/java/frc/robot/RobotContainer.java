// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
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
import frc.robot.subsystems.VisionSys;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CORAL;
import frc.robot.Constants.OPERATOR;
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


  private final CommandXboxController m_controlController = new CommandXboxController(Constants.OPERATOR.CONTROL_CONTROLLER_PORT);
  private final CommandXboxController m_driver2 = new CommandXboxController(Constants.OPERATOR.DRIVE_CONTROLLER_PORT_2);

 // private final Joystick driveJoystick = new Joystick(Constants.OPERATOR.DRIVE_CONTROLLER_PORT);

  //Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
  SwerveInputStream driveAngularVelocity = 
    SwerveInputStream.of(drivebase.getSwerveDrive(),
                          () -> m_driver2.getLeftY() * 1, //LY, 1
                          () -> m_driver2.getLeftX() * 1) //LX, 0
                      .withControllerRotationAxis(m_driver2::getRightX) //Rotate 2
                      .deadband(OPERATOR.DEADBAND)
                      .scaleTranslation(0.8)
                      .scaleRotation(0-0.4)
                      .allianceRelativeControl(false);

  // Clones the angular velocity input stream and converts it to a robotRelative input stream.
  SwerveInputStream driveRobotOriented = 
    driveAngularVelocity.copy().robotRelative(true)
                          .allianceRelativeControl(false);
  
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

      // coral position
      m_controlController.pov(0).onTrue(new ToCL4(m_elevatorSys, m_manpulatorPitchSys)); // up
      m_controlController.pov(90).onTrue(new ToCL3(m_elevatorSys, m_manpulatorPitchSys)); // right
      m_controlController.pov(180).onTrue(new ToCL1(m_elevatorSys, m_manpulatorPitchSys)); // down
      m_controlController.pov(270).onTrue(new ToCL2(m_elevatorSys, m_manpulatorPitchSys)); // left
      m_controlController.button(9).onTrue(new ToHome(m_elevatorSys, m_manpulatorPitchSys)); // left stick button

      // coral manipultion (replace with automatic single button)
      m_controlController.leftBumper().whileTrue(new CManipulate(m_coralManipulatorSys, CORAL.INTAKE_SPEED)); // left bumper
      m_controlController.button(7).whileTrue(new CManipulate(m_coralManipulatorSys, CORAL.FRONT_OUTTAKE_SPEED));
      m_controlController.button(11).whileTrue(new CManipulate(m_coralManipulatorSys, -CORAL.BACK_OUTTAKE_SPEED));//left stick

      //algae manipulation
      m_controlController.rightBumper().whileTrue(new AManipulate(m_algaeManipulatorSys, 0.3)); // left bumper
      m_controlController.button(8).whileTrue(new AManipulate(m_algaeManipulatorSys, -1));
      m_algaeManipulatorSys.setDefaultCommand(new AManipulate(m_algaeManipulatorSys, 0));

      // algae position
      m_controlController.button(4).onTrue(new ToANet(m_elevatorSys, m_manpulatorPitchSys)); // y
      m_controlController.button(1).onTrue(new ToAL1(m_elevatorSys, m_manpulatorPitchSys)); // x
      m_controlController.button(3).onTrue(new ToAL2(m_elevatorSys, m_manpulatorPitchSys)); // b
      m_controlController.button(2).onTrue(new ToAProcessor(m_elevatorSys, m_manpulatorPitchSys)); // a
      m_controlController.button(10).onTrue(new ToAGround(m_elevatorSys, m_manpulatorPitchSys)); // start

    m_driver2.button(1).onTrue(new ToClimbHome(m_climbSys)); //A
    m_driver2.button(2).onTrue(new ToClimbReady(m_climbSys)); //B
    m_driver2.button(4).onTrue(new ToClimbLatched(m_climbSys)); //Y

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
