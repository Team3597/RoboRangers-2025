// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static boolean DEBUG_MODE = true; //enables debug sysouts
    public static boolean MANIPULATOR_CLEAR = false; //are we good to change manipulator pitch?
  }

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);

  public static class OPERATOR {
    public static final int DRIVE_CONTROLLER_PORT = 0;
    public static final int CONTROL_CONTROLLER_PORT = 1;
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.3;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 3; // was 6
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
    public static final int ALGAE_MANIPULATOR = 12;
    public static final int CORAL_MANIPULATOR = 13;

    public static final int CLIMB = 14;
  }

  public static class DIO {

    public static final int CORAL_LIMIT = 0;
    
  }

  public static class MOTION {
    public static final double ALGAE_INTAKE_RPM = 60;
    public static final double ALGAE_OUTTAKE_RPM = 60;
    //potentially differ speeds for different levels later
    public static final double CORAL_INTAKE_SPEED = 0.2;
    public static final double CORAL_FRONT_OUTTAKE_SPEED = 0.2;
    public static final double CORAL_BACK_OUTTAKE_SPEED = 0.2;
  }

  public static class PID {
    public static final double ALGAE_P = 0.1;
    public static final double ALGAE_I = 0;
    public static final double ALGAE_D = 0;
    public static final double ALGAE_FF = 0.002; // 1/KV, 1/473
    public static final double ALGAE_MIN = -0.2;
    public static final double ALGAE_MAX = 0.2;
    //figure out graphing my encoder data so i can tune these? since i can't observe velocity easily

    
    public static double PITCH_P = 3;
    public static double PITCH_I = 0;
    public static double PITCH_D = 0;
    public static final double PITCH_MIN = -1;
    public static final double PITCH_MAX = 1;

    public static final double ELEVATOR_P = 0.5;
    public static final double ELEVATOR_I = 0;
    public static final double ELEVATOR_D = 0.01;
    public static final double ELEVATOR_MIN = -0.5;
    public static final double ELEVATOR_MAX = 0.5;
    public static final double ELEVATOR_FF = 0;

    public static final double CLIMB_P = 0;
    public static final double CLIMB_I = 0;
    public static final double CLIMB_D = 0;
    public static final double CLIMB_MIN = 0;
    public static final double CLIMB_MAX = 0;
  }

  public static class UTIL {
  }

  public static final class DrivebaseConstants
  {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class ELEVATOR {
    public static final int AMP_LIMIT = 30;

    public static final double ELEVATOR_COUNT_OFFSET = 0;
    public static final double ELEVATOR_MAX_HEIGHT = 53;
    public static final double ELEVATOR_MAX_COUNTS = 56;

    public static final double HOME = 0; // home height
    public static final double CLEAR = 0; // clearance height for manipulator
    
    public static final double AL1 = 0; // algae l1, l2, net
    public static final double AL2 = 0;
    public static final double ANET = 0;

    public static final double CL1 = 10; // coral l1, l2, l3, l4
    public static final double CL2 = 20;
    public static final double CL3 = 30;
    public static final double CL4 = 40;
    
    public static final double DEADBAND = 0;
  }

  public static class MANIPULATOR {
    public static final int AMP_LIMIT = 20;

    public static final double NEO_CPR = 42;
    public static final double THROUGHBORE_CPR = 8192;
    public static final double MANIPULATOR_PIVOT_OFFSET = 0;

    public static final double MANIPULATOR_MIN_PITCH = 0.57;
    public static final double MANIPULATOR_MAX_PITCH = 0.99;

    public static final double HOME = 0.57; // home pitch
    public static final double UNSTICK = 0.6; // pitch to unstick coral

    public static final double AGROUND = 0.7; // algae ground, processor, reef (l1, l2), net
    public static final double APROCESSOR = 0.75;
    public static final double AREEF = 0.75;
    public static final double ANET = 0.99;

    public static final double CLOW = HOME; // coral low (l1, l2, l3), high (l4)
    public static final double CHIGH = 0.99;

    public static final double DEADBAND = 0;
  }

  public static class CLIMB {
    public static final int AMP_LIMIT = 10;
    
    public static final double CLIMB_MIN_PITCH = 0;
    public static final double CLIMB_MAX_PITCH = 1;

    public static final double HOME = 0;
    public static final double LATCHED = 0;
    public static final double READY = 0;

    public static final double DEADBAND = 0;
  }

  public static class CAMERA {
    public static final String CAMERA_NICKNAME = "2597Camera"; // camera nickname (needs to be updated)
    
    public static final double CAMERA_HEIGHT = 0;
    public static final double CAMERA_PITCH = 0;

  }
}
