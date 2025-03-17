// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.AlgaeManipulatorSys;
import frc.robot.subsystems.ClimbSys;
import frc.robot.subsystems.CoralManipulatorSys;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ManipulatorPitchSys;
import frc.robot.subsystems.StateSys;
import frc.robot.subsystems.StateSys.scoring;
import frc.robot.subsystems.VisionSys;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OPERATOR;
import frc.robot.commands.ManipulateObject;
import frc.robot.commands.SetScoring;
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

  private static final AlgaeManipulatorSys m_algaeManipulatorSys = new AlgaeManipulatorSys();
  private static final CoralManipulatorSys m_coralManipulatorSys = new CoralManipulatorSys();
  private static final ElevatorSys m_elevatorSys = new ElevatorSys();
  private static final ManipulatorPitchSys m_manpulatorPitchSys = new ManipulatorPitchSys();
  private static final ClimbSys m_climbSys = new ClimbSys();
  private static final VisionSys m_visionSys = new VisionSys();
  private static final StateSys m_stateSys = new StateSys();


  private final CommandXboxController m_gunnerController = new CommandXboxController(OPERATOR.GUNNER_CONTROLLER_PORT);
  private final CommandXboxController m_driveController = new CommandXboxController(OPERATOR.DRIVE_CONTROLLER_PORT);

 // private final Joystick driveJoystick = new Joystick(Constants.OPERATOR.DRIVE_CONTROLLER_PORT);

  private final SendableChooser<Command> autoChooser; // Auto chooser for path planner (part of recommended method of choosing an auto).

  //Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
  SwerveInputStream driveAngularVelocity = 
    SwerveInputStream.of(drivebase.getSwerveDrive(),
                          () -> m_driveController.getLeftY() * 1, //LY, 1
                          () -> m_driveController.getLeftX() * 1) //LX, 0
                      .withControllerRotationAxis(m_driveController::getRightX) //Rotate 2
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
    // Named commands:
    NamedCommands.registerCommand("test_named_command", Commands.print("I EXIST"));
    NamedCommands.registerCommand("test_subsystems", SetScoring(scoring.CL3));

    // Configure the trigger bindings
    configureBindings();
    
    DriverStation.silenceJoystickConnectionWarning(true);

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
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

  // lazy method to get full set scoring commands from just the target
  private SetScoring SetScoring(StateSys.scoring target) {
    return new SetScoring(target, m_stateSys, m_elevatorSys, m_manpulatorPitchSys);
  }

  private ManipulateObject ManipulateObject() {
    return new ManipulateObject(m_stateSys, m_coralManipulatorSys, m_algaeManipulatorSys);
  }

  private void configureBindings() {


    // coral position
    m_gunnerController.pov(0).onTrue(SetScoring(scoring.CL4)); // up
    m_gunnerController.pov(90).onTrue(SetScoring(scoring.CL3)); // right
    m_gunnerController.pov(180).onTrue(SetScoring(scoring.CL1)); // down
    m_gunnerController.pov(270).onTrue(SetScoring(scoring.CL2)); // left
    m_gunnerController.button(9).onTrue(SetScoring(scoring.Home)); // left stick button


    // automatic manipulation
    m_gunnerController.leftBumper().whileTrue(ManipulateObject()); // left bumper


    // algae position
    m_gunnerController.button(4).onTrue(SetScoring(scoring.ANet)); // y
    m_gunnerController.button(1).onTrue(SetScoring(scoring.AL1)); // x
    m_gunnerController.button(3).onTrue(SetScoring(scoring.AL2)); // b
    m_gunnerController.button(2).onTrue(SetScoring(scoring.AGround)); // a
    m_gunnerController.button(10).onTrue(SetScoring(scoring.AProcessor)); // start


    // m_driveController.button(1).onTrue(new ToClimbHome(m_climbSys)); //A
    // m_driveController.button(2).onTrue(new ToClimbReady(m_climbSys)); //B
    // m_driveController.button(4).onTrue(new ToClimbLatched(m_climbSys)); //Y

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
      m_gunnerController.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      m_gunnerController.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveRobotOrientedAngularVelocity); // Overrides drive command above!

      m_gunnerController.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      m_gunnerController.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      m_gunnerController.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      m_gunnerController.back().whileTrue(drivebase.centerModulesCommand());
      m_gunnerController.leftBumper().onTrue(Commands.none());
      m_gunnerController.rightBumper().onTrue(Commands.none());
    } else
    {
      // m_gunnerController.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      // m_gunnerController.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      // m_gunnerController.b().whileTrue(
      //     drivebase.driveToPose(
      //         new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
      //                         );
      m_gunnerController.start().whileTrue(Commands.none());
      m_gunnerController.back().whileTrue(Commands.none());
      m_gunnerController.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      m_gunnerController.rightBumper().onTrue(Commands.none());

    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   // return autoChooser.getSelected();

    // check if my command ends correctly
    return new SequentialCommandGroup(
      SetScoring(scoring.CL2),
      new InstantCommand(() -> System.out.println("test"))
    );
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
