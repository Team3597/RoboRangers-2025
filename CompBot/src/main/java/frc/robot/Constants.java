// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
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

    public static final int ELEVATOR_MAIN = 9; //left
    public static final int ELEVATOR_SLAVE = 10; //right

    public static final int MANIPULATOR_PITCH = 11;
    public static final int ALGAE_MANIPULATOR = 12;
    public static final int CORAL_MANIPULATOR = 13;
  }

  public static class Motion {
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
    public static final double ALGAE_MIN = -1;
    public static final double ALGAE_MAX = 1;
    //figure out graphing my encoder data so i can tune these? since i can't observe velocity easily

    
    public static final double PITCH_P = 0;
    public static final double PITCH_I = 0;
    public static final double PITCH_D = 0;
    public static final double PITCH_MIN = -1;
    public static final double PITCH_MAX = 1;
    public static final double ELEVATOR_P = 0;
    public static final double ELEVATOR_I = 0;
    public static final double ELEVATOR_D = 0;
    public static final double ELEVATOR_MIN = 0;
    public static final double ELEVATOR_MAX = 0;
    public static final double ELEVATOR_FF = 0;
  }

  public static class UTIL {
  }

  public static class ELEVATOR {
    public static final double ELEVATOR_COUNT_OFFSET = 0;
    public static final double ELEVATOR_MAX_HEIGHT = 30;
    public static final double ELEVATOR_MAX_COUNTS = 8000;

    public static final double HOME = 0; // home height
    public static final double CLEAR = 0; // clearance height for manipulator

    public static final double APROCESSOR = 0; // algae processor, l1, l2, net
    public static final double AL1 = 0;
    public static final double AL2 = 0;
    public static final double ANET = 0;

    public static final double CL1 = 0; // coral l1, l2, l3, l4
    public static final double CL2 = 0;
    public static final double CL3 = 0;
    public static final double CL4 = 0;
  }

  public static class MANIPULATOR {
    public static final double NEO_CPR = 42;
    public static final double THROUGHBORE_CPR = 8192;
    public static final double MANIPULATOR_PIVOT_OFFSET = 0;

    public static final double MANIPULATOR_MIN_PITCH = 0;
    public static final double MANIPULATOR_MAX_PITCH = 1;

    public static final double HOME = 0; // home pitch
    public static final double UNSTICK = 0; // pitch to unstick coral

    public static final double AGROUND = 0; // algae ground, processor, reef (l1, l2), net
    public static final double APROCESSOR = 0;
    public static final double AREEF = 0;
    public static final double ANET = 0;

    public static final double CLOW = 0; // coral low (l1, l2, l3), high (l4)
    public static final double CHIGH = 0;
  }
}
