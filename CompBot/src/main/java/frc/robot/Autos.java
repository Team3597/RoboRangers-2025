// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot;

// import choreo.auto.AutoFactory;
// import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// public class Autos {

//     private SwerveSubsystem swerveSubsystem;
//     private final AutoFactory autoFactory;

//     public Autos(SwerveSubsystem swerve) {
//         this.swerveSubsystem = swerve;
//         autoFactory = new AutoFactory(
//             swerveSubsystem::getPose, // A function that returns the current robot pose
//             swerveSubsystem::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
//             swerveSubsystem::driveToPose, // The drive subsystem trajectory follower 
//             true, // If alliance flipping should be enabled 
//             swerveSubsystem // The drive subsystem
//         );
        
//     }

// }
