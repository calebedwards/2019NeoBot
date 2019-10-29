/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  public static final int leftMotor = 2;
  public static final int rightMotor = 3;

  public static final int encoderFL = 1;
  public static final int encoderBL = 2;
  public static final int encoderFR = 3;
  public static final int encoderBR = 0;

  public static final int driveMotorFL = 2;
  public static final int angleMotorFL = 3;
  public static final int driveMotorBL = 4;
  public static final int angleMotorBL = 5;
  public static final int driveMotorFR = 6;
  public static final int angleMotorFR = 7;
  public static final int driveMotorBR = 8;
  public static final int angleMotorBR = 9;
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
