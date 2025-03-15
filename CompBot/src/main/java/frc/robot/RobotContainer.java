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
import frc.robot.subsystems.VisionSys;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CORAL;
import frc.robot.Constants.OPERATOR;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import lib.frc3597.state.State;
import lib.frc3597.state.StateMachine;
import lib.frc3597.state.Transition;

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
    // Configure the trigger bindings
    configureBindings();
    
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

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
  private void configureBindings() {

      // coral position
      m_gunnerController.pov(0).onTrue(new ToCL4(m_elevatorSys, m_manpulatorPitchSys)); // up
      m_gunnerController.pov(90).onTrue(new ToCL3(m_elevatorSys, m_manpulatorPitchSys)); // right
      m_gunnerController.pov(180).onTrue(new ToCL1(m_elevatorSys, m_manpulatorPitchSys)); // down
      m_gunnerController.pov(270).onTrue(new ToCL2(m_elevatorSys, m_manpulatorPitchSys)); // left
      m_gunnerController.button(9).onTrue(new ToHome(m_elevatorSys, m_manpulatorPitchSys)); // left stick button


      // coral manipultion (replace with automatic single button)
      m_gunnerController.leftBumper().whileTrue(new CManipulate(m_coralManipulatorSys, CORAL.INTAKE_SPEED)); // left bumper
      m_gunnerController.button(7).whileTrue(new CManipulate(m_coralManipulatorSys, CORAL.FRONT_OUTTAKE_SPEED));
      m_gunnerController.button(11).whileTrue(new CManipulate(m_coralManipulatorSys, -CORAL.BACK_OUTTAKE_SPEED));//left stick

      //algae manipulation
      m_gunnerController.rightBumper().whileTrue(new AManipulate(m_algaeManipulatorSys, 0.3)); // left bumper
      m_gunnerController.button(8).whileTrue(new AManipulate(m_algaeManipulatorSys, -1));
      m_algaeManipulatorSys.setDefaultCommand(new AManipulate(m_algaeManipulatorSys, 0));

      // algae position
      m_gunnerController.button(4).onTrue(new ToANet(m_elevatorSys, m_manpulatorPitchSys)); // y
      m_gunnerController.button(1).onTrue(new ToAL1(m_elevatorSys, m_manpulatorPitchSys)); // x
      m_gunnerController.button(3).onTrue(new ToAL2(m_elevatorSys, m_manpulatorPitchSys)); // b
      m_gunnerController.button(2).onTrue(new ToAProcessor(m_elevatorSys, m_manpulatorPitchSys)); // a
      m_gunnerController.button(10).onTrue(new ToAGround(m_elevatorSys, m_manpulatorPitchSys)); // start


    m_driveController.button(1).onTrue(new ToClimbHome(m_climbSys)); //A
    m_driveController.button(2).onTrue(new ToClimbReady(m_climbSys)); //B
    m_driveController.button(4).onTrue(new ToClimbLatched(m_climbSys)); //Y

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
    return autoChooser.getSelected();

    // return drivebase.driveToDistanceCommand(5, 1.5).withTimeout(2); // Old auto command
    //Autos.exampleAuto(m_exampleSubsystem);
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
