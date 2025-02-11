// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(6);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class ElevatorConstants
  {
    public static final int CHANNEL_A = 4;  // Fio Azul
    public static final int CHANNEL_B = 5;  // Fio Amarelo
    
    public static final int ID_LEFT_MOTOR = 14;
    public static final int ID_RIGHT_MOTOR = 15;
    
    public static final double L1_HEIGHT = 0;
    public static final double L2_HEIGHT = 15;
    public static final double L3_HEIGHT = 53;
    public static final double L4_HEIGHT = 88;

    public static final double MAX_ENCODER = 5120;
    
  }

  public static class IntakeConstants
  {

    public static final int ID_RIGHT_ALGAE_MOTOR = 16;
    public static final int ID_LEFT_ALGAE_MOTOR = 17;

    public static final int ID_CORAL_MOTOR = 18;

    public static final int ID_WRIST_MOTOR = 19;

    public static final double POSITION_ANGLE_WRIST = 2.4;
    public static final double POSITION_ANGLE_WRIST_L1 = 3.0;

    
  }

  public static class ButtonConstants
  {
    public static final int COLLECT_ALGAE = 1;
    public static final int COLLECT_CORAL = 1;

    public static final int SHOOTER_ALGAE = 2;
    public static final int SHOOTER_CORAL = 3;

    public static final int GO_TO_L1 = 4;
    public static final int GO_TO_L2 = 5;
    public static final int GO_TO_L3 = 6;
    public static final int GO_TO_L4 = 7;

    public static final int END_GAME = 8;

  }

}
