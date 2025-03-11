// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class GLOBAL {
    public static boolean DEBUG_MODE = false; //enables debug sysouts
    public static boolean MANIPULATOR_CLEAR = false; //are we good to change manipulator pitch?

    public static boolean DISABLE_ELEVATOR = false;
    public static boolean DISABLE_MANIPULATOR_PITCH = false;
  }

  public static class PROPERTIES {
    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED  = Units.feetToMeters(14.5);
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OPERATOR {
    public static final int DRIVE_CONTROLLER_PORT = 0;
    public static final int CONTROL_CONTROLLER_PORT = 1;
    public static final int DRIVE_CONTROLLER_PORT_2 = 2;
    
    public static final double DEADBAND = 0.01;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 3; // was 6
  }

  public static class CAN {
    public static final int FRONT_LEFT_DRIVE = 1;
    public static final int FRONT_LEFT_ROT = 2;
    public static final int FRONT_RIGHT_DRIVE = 3;
    public static final int FRONT_RIGHT_ROT = 4;
    public static final int BACK_LEFT_DRIVE = 5;
    public static final int BACK_LEFT_ROT = 6;
    public static final int BACK_RIGHT_DRIVE = 7;
    public static final int BACK_RIGHT_ROT = 8;

    public static final int ELEVATOR_MAIN = 9; //left, - is up
    public static final int ELEVATOR_SLAVE = 10; //right + is up

    public static final int MANIPULATOR_PITCH = 11; // + is up
    public static final int ALGAE_MANIPULATOR = 12; // + is in
    public static final int CORAL_MANIPULATOR = 13;

    public static final int CLIMB = 14;
  }

  public static class DIO {
    public static final int CORAL_LIMIT = 0;
  }

  public static class ALGAE {
    public static class PID {
      public static final double P = 0.001;
      public static final double I = 0;
      public static final double D = 0.01;
      public static final double FF = 0.002; // 1/KV, 1/473
      public static final double MIN = -0.2;
      public static final double MAX = 0.2;
    }

    public static final double INTAKE_RPM = 6000;
    public static final double OUTTAKE_RPM = 6000;
  }

  public static class CORAL {
    //potentially differ speeds for different levels later
    public static final double INTAKE_SPEED = 1;
    public static final double FRONT_OUTTAKE_SPEED = 1;
    public static final double BACK_OUTTAKE_SPEED = 1;
  }

  public static class ELEVATOR {
    public static final int AMP_LIMIT = 30;

    public static class PID {
      public static final double P = 0.25;
      public static final double I = 0.0;
      public static final double D = 0.02;
      public static final double MIN = -1;
      public static final double MAX = 1;
      public static final double FF = 0.022;
      //0.022
      public static final double MAX_A = 400;
      //618.55;
      public static final double MAX_V = 30;
      //25.08;

      // public static final double P = 2;
      // public static final double I = 0.001;
      // public static final double D = 0.2;
      // public static final double MIN = -1;
      // public static final double MAX = 1;
      // public static final double FF = 0.022;
      // public static final double MAX_A = 3000;
      // public static final double MAX_V = 10000;
    }

    public static final double COUNT_OFFSET = 0;
    public static final double MAX_HEIGHT = 53;
    public static final double MAX_COUNTS = 56;

    public static final double HOME = .5; // home height
    public static final double CLEAR = 2; // clearance height for manipulator

    public static final double AL1 = 26.5; // algae l1, l2, net
    public static final double AL2 = 41;
    public static final double ANET = 52.5;
    public static final double APROCESSOR = 2;

    public static final double CL1 = 14.5; // coral l1, l2, l3, l4
    public static final double CL2 = 26.5;
    public static final double CL3 = 42.8;
    public static final double CL4 = 53;
    
    public static final double DEADBAND = 0.25;
  }

  public static class MANIPULATOR {
    public static final int AMP_LIMIT = 20;
    public static final int CORAL_AMP_LIMIT = 40;
    public static final int ALGAE_AMP_LIMIT = 30;

    public static class PID {
      public static double P = 3;
      public static double I = 0;
      public static double D = 0.01;
      public static final double MIN = -0.5;
      public static final double MAX = 0.5;

    }

    public static final double MANIPULATOR_PIVOT_OFFSET = 0;
    //encoder got screwed so zeroed + am lazy

    public static final double MANIPULATOR_MIN_PITCH = 0;
    //0.57
    public static final double MANIPULATOR_MAX_PITCH = 0.5;
    //0.99

    public static final double HOME = 0; // home pitch
    //0.57.
 
    public static final double UNSTICK = 0.03; // pitch to unstick coral

    public static final double AGROUND = 0.13; // algae ground, processor, reef (l1, l2), net
    public static final double APROCESSOR = 0.18;
    public static final double AREEF = 0.18;
    public static final double ANET = 0.42;

    public static final double CLOW = HOME; // coral low (l1, l2, l3), high (l4)
    public static final double CHIGH = 0.48;
    public static final double CL1 = 0.1;

    public static final double DEADBAND = 0.1;
  }

  public static class CLIMB {
    public static final int AMP_LIMIT = 40;

    public static class PID {
      public static final double P = 3;
      public static final double I = 0.1;
      public static final double D = 0;
      public static final double MIN = -1;
      public static final double MAX = 1;

      public static final double P_CLIMB = 0;
      public static final double I_CLIMB = 0;
      public static final double D_CLIMB = 0;
      public static final double MIN_CLIMB = 0;
      public static final double MAX_CLIMB = 0;
    }
    
    public static final double CLIMB_MIN_PITCH = 0.44;
    public static final double CLIMB_MAX_PITCH = 0.44;

    public static final double HOME = 0.375;
    public static final double LATCHED = 0.5;
    public static final double READY = 0.125;


    public static final double DEADBAND = 0.5;
  }

  public static class CAMERA {
    public static final String CAMERA_NICKNAME = "Team3597Camera"; // camera nickname (needs to be updated)
    
    public static final double CAMERA_X_FROM_ROBOT_CENTER_INCHES = 0; // the sign of these might have to be experimented with
    public static final double CAMERA_Y_FROM_ROBOT_CENTER_INCHES = 0;
    public static final double CAMERA_HEIGHT_INCHES = 0;
    public static final Translation3d CAMERA_TRANSLATION_3D = new Translation3d(Units.inchesToMeters(CAMERA_X_FROM_ROBOT_CENTER_INCHES), Units.inchesToMeters(CAMERA_Y_FROM_ROBOT_CENTER_INCHES), Units.inchesToMeters(CAMERA_HEIGHT_INCHES));
    
    public static final double CAMERA_ROLL_DEGREES = 0; // (spin clockwise/counterclockwise facing camera)
    public static final double CAMERA_PITCH_DEGREES = 0; // (tilt up/down)
    public static final double CAMERA_YAW_DEGREES = 0; // (angle left/right)
    public static final Rotation3d CAMERA_ROTATION_3D = new Rotation3d(Units.degreesToRadians(CAMERA_ROLL_DEGREES), Units.degreesToRadians(CAMERA_PITCH_DEGREES), Units.degreesToRadians(CAMERA_YAW_DEGREES));

    public static final Transform3d CAMERA_TRANSFORM_3D = new Transform3d(CAMERA_TRANSLATION_3D, CAMERA_ROTATION_3D);

    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public static final PhotonPoseEstimator PHOTON_POSE_ESTIMATOR = new PhotonPoseEstimator(CAMERA.APRIL_TAG_FIELD_LAYOUT, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CAMERA.CAMERA_TRANSFORM_3D);
  }
}
