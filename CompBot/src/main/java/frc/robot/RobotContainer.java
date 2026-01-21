// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/* imports give the file access to external classes
   only import what you actually need to keep code clean and encapsulated */

// library imports; wpilib imports the libraries you need as you type their commands automatically
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.io.File;
import swervelib.SwerveInputStream;

/* subsystem imports; gives this file access to subsystem classes 
to make an instance of them to pass to commands */
import frc.robot.subsystems.StateSys;
import frc.robot.subsystems.StateSys.scoring;
import frc.robot.subsystems.manipulator.AlgaeManipulatorSys;
import frc.robot.subsystems.manipulator.CoralManipulatorSys;
import frc.robot.subsystems.manipulator.ManipulatorPitchSys;
import frc.robot.subsystems.superstructure.ClimbSys;
import frc.robot.subsystems.superstructure.ElevatorSys;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// command imports; gives this file access to commands to run when buttons are pressed
import frc.robot.commands.ManipulateObject;
import frc.robot.commands.SetClimbing;
import frc.robot.commands.SetScoring;
import frc.robot.commands.climb;

// constants imports; gives this file access to necessary subclasses of constant values
import frc.robot.Constants.OPERATOR;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  /* make an object of each subsystem class, following format m_subsystem 
     (important for organization and to distinguish from class itself) */
  private static final AlgaeManipulatorSys m_algaeManipulatorSys = new AlgaeManipulatorSys();
  private static final CoralManipulatorSys m_coralManipulatorSys = new CoralManipulatorSys();
  private static final ElevatorSys m_elevatorSys = new ElevatorSys();
  private static final ManipulatorPitchSys m_manpulatorPitchSys = new ManipulatorPitchSys();
  private static final ClimbSys m_climbSys = new ClimbSys();
  private static final StateSys m_stateSys = new StateSys();

  // swerve subsystem instatiation is a little different since this code comes from the swerve library
  private final SwerveSubsystem drivebase = 
    new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  // make an object for each controller attached to the port specified in operator constants
  private final CommandXboxController m_gunnerController = new CommandXboxController(OPERATOR.GUNNER_CONTROLLER_PORT);
  private final CommandXboxController m_driveController = new CommandXboxController(OPERATOR.DRIVE_CONTROLLER_PORT);

  // make an object for the dropdown that you can use to pick a pathplanner auto (may not work lol)
  private final SendableChooser<Command> autoChooser; 

  // prewritten swerve code, don't significantly mess with unless you've read the swerve docs and really know what you're doing
  // Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
  SwerveInputStream driveAngularVelocity = 
    SwerveInputStream.of(drivebase.getSwerveDrive(),
                          () -> m_driveController.getRawAxis(1) * 1, //LY, 1
                          () -> m_driveController.getRawAxis(0) * 1) //LX, 0
                      .withControllerRotationAxis(m_driveController::getRightX) //Rotate 2
                      .deadband(OPERATOR.DEADBAND) // ignores tiny controller movement
                      .scaleTranslation(0.5) // change to adjust translation speed
                      .scaleRotation(-0.7) // change to adjust rotation speed
                      .allianceRelativeControl(false); // KEEP THIS OFF!!! UNLESS THIS GETS FIXED!! 
                                                               // (should work on fixing but TEST FIRST)

    // inverted drive setup for when alliance swaps. we ignore this
    SwerveInputStream driveAngularVelocityBlue = 
    SwerveInputStream.of(drivebase.getSwerveDrive(),
                          () -> m_driveController.getRawAxis(1) * -1, //LY, 1
                          () -> m_driveController.getRawAxis(0) * -1) //LX, 0
                      .withControllerRotationAxis(m_driveController::getRightX) //Rotate 2
                      .deadband(OPERATOR.DEADBAND)
                      .scaleTranslation(0.8)
                      .scaleRotation(0-0.4)
                      .cubeTranslationControllerAxis(true)
                      .allianceRelativeControl(false);

  // more prewritten swerve
  // Clones the angular velocity input stream and converts it to a robotRelative input stream.
  SwerveInputStream driveRobotOriented = 
    driveAngularVelocity.copy().robotRelative(true)
                          .allianceRelativeControl(false);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // establish all the commands that that show up and can be used in pathplanner
    NamedCommands.registerCommand("test_named_command", Commands.print("I EXIST"));
    NamedCommands.registerCommand("CL4", SetScoring(scoring.CL4));
    NamedCommands.registerCommand("CL3", SetScoring(scoring.CL3));
    NamedCommands.registerCommand("CL2", SetScoring(scoring.CL2));
    NamedCommands.registerCommand("CL1", SetScoring(scoring.CL1));
    NamedCommands.registerCommand("home", SetScoring(scoring.Home));

    // this is multiple commands built inline from methods chained into one command
    // read the docs for this because i suck at it
    NamedCommands.registerCommand("dumber_manipulate", 
      new SequentialCommandGroup(
        Commands.runOnce(() -> m_coralManipulatorSys.backOuttakeCoral()),
        (Commands.waitSeconds(2)),
        (Commands.runOnce(() -> m_coralManipulatorSys.stop()))
        )
    );
    
    // run methods further below to bind buttons to commands and set commands to run constantly
    configureBindings();
    setDefaultCommands();
    
    DriverStation.silenceJoystickConnectionWarning(true);

    // Build an auto chooser dropdown and specify the default auto by its name
    autoChooser = AutoBuilder.buildAutoChooser("test_auto_2");

    // Send the auto chooser dropdown to the dashboard to be interacted with
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

  // commands need to be passed all relevant subsystems as well as any parameters
  // since typing all this for every similar binding sucks these methods wrap all that into a single method
  private SetScoring SetScoring(StateSys.scoring target) {
    return new SetScoring(target, m_stateSys, m_elevatorSys, m_manpulatorPitchSys);
  }

  private ManipulateObject ManipulateObject() {
    return new ManipulateObject(m_stateSys, m_coralManipulatorSys, m_algaeManipulatorSys);
  }

  // relic from attempts at using pid control on the climb
  // testing this blew up a versaplanetary so we switched to a dumber strategy and thus this is unused
  private SetClimbing SetClimbing(StateSys.climbing target) {
    return new SetClimbing(target, m_stateSys, m_climbSys);
  }

  /* binds controller buttons to commands. note the use of specific button ids. you can specify buttons 
     by their conventional names but since we use cheap knockoff controllers sometimes these names dont 
     match the actual buttons and thus just using ids is safer. make sure you comment the conventional
     button name so that you know what to actually press though */
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

    // old pid controlled climb
    // m_gunnerController.button(6).onTrue(SetClimbing(climbing.Home)); // right bumper
    // m_gunnerController.button(8).onTrue(SetClimbing(climbing.Ready)); // right trigger
    // m_gunnerController.button(12).onTrue(SetClimbing(climbing.Latched)); // right stick

    /* this is a more typical example of what a command binding looks like, making a new object of the 
       climb command and passing it parameters and the instances of the subsystems involved */
    m_gunnerController.button(6).whileTrue(new climb(0.3, m_climbSys)); // right bumper
    m_gunnerController.button(8).whileTrue(new climb(-0.5, m_climbSys)); // right trigger
    m_gunnerController.button(12).whileTrue(new climb(0.7, m_climbSys)); // right stick

    m_gunnerController.button(11).onTrue(
      new SequentialCommandGroup(
        Commands.runOnce(() -> m_coralManipulatorSys.manipulateCoral(1)),
        Commands.waitSeconds(3),
        Commands.runOnce(() -> m_coralManipulatorSys.stop())
      )
    );
    
    // toggle driving relative to the robot (tank drive style) instead of to the field
    // Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    // m_driveController.button(6).whileTrue(driveRobotOrientedAngularVelocity);


  }

  private void setDefaultCommands() {
    // stuff to invert field oriented driving when alliance swaps. BROKEN AND NEEDS FIXED!!
    // var alliance = DriverStation.getAlliance();
    // if (alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false){ //We're Red
    //   Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    //   drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    // }
    // else{ // We're Blue
    //   Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocityBlue);
    //   drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    // }

    // Use the default field-oriented angular velocity drive command
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // sends command selected from autochooser dropdown
    return autoChooser.getSelected();
  }

  // used to unbrake motors when robot is disabled
  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
