/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.Console;
import java.util.logging.Level;
import java.util.logging.Logger;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.cmdHolonomicDrive;

public class SwerveDriveSubsystem extends HolonomicDriveTrain {
  // wheel base is the distance between the center of the left and right wheels
  // track width is the distance between the center of the front and back wheels
  public static final double WHEELBASE = 18; // Swerve bot: 14.5 Comp bot: 20.5
  public static final double TRACKWIDTH = 18; // Swerve bot: 13.5 Comp bot: 25.5

  public static final double WIDTH = 25; // Swerve bot: 20 Comp bot: 37
  public static final double LENGTH = 25; // Swerve bot: 19 Comp bot: 32

  /*
   * 0 is Front Right 1 is Front Left 2 is Back Left 3 is Back Right
   */
  private SwerveModule[] mSwerveModules;

  private AHRS mNavX = new AHRS(SPI.Port.kMXP, (byte) 200);

  public SwerveDriveSubsystem() {
    super(WIDTH, LENGTH);
    zeroGyro();

    mSwerveModules = new SwerveModule[] {
        new SwerveModule(1, RobotMap.angleMotorFL, RobotMap.driveMotorFL, RobotMap.encoderFL, 34.65),
        new SwerveModule(2, RobotMap.angleMotorBL, RobotMap.driveMotorBL, RobotMap.encoderBL, 84.675),
        new SwerveModule(3, RobotMap.angleMotorFR, RobotMap.driveMotorFR, RobotMap.encoderFR, 255.225),
        new SwerveModule(4, RobotMap.angleMotorBR, RobotMap.driveMotorBR, RobotMap.encoderBR, 129.974)

    };
    mSwerveModules[2].setDriveInverted(true);
    mSwerveModules[3].setDriveInverted(true);

    for (SwerveModule module : mSwerveModules) {
      module.setTargetAngle(0);
      module.setDriveGearRatio(5.7777);
      module.setDriveWheelRadius(module.getDriveWheelRadius() * 1.05);
      module.setMotionConstraints(getMaxAcceleration(), getMaxVelocity());
    }
  }

  public double[] calculateSwerveModuleAngles(double forward, double strafe, double rotation) {
    if (isFieldOriented()) {
      double angleRad = Math.toRadians(getGyroAngle());
      double temp = forward * Math.cos(angleRad) + strafe * Math.sin(angleRad);
      strafe = -forward * Math.sin(angleRad) + strafe * Math.cos(angleRad);
      forward = temp;
    }

    double a = strafe - rotation * (WHEELBASE / TRACKWIDTH);
    double b = strafe + rotation * (WHEELBASE / TRACKWIDTH);
    double c = forward - rotation * (TRACKWIDTH / WHEELBASE);
    double d = forward + rotation * (TRACKWIDTH / WHEELBASE);

    return new double[] { Math.atan2(b, c) * 180 / Math.PI, Math.atan2(b, d) * 180 / Math.PI,
        Math.atan2(a, d) * 180 / Math.PI, Math.atan2(a, c) * 180 / Math.PI };
  }

  public AHRS getNavX() {
    return mNavX;
  }

  public double getGyroAngle() {
    double angle = mNavX.getAngle() - getAdjustmentAngle();
    angle %= 360;
    if (angle < 0) {
      angle += 360;
    }

    return angle;

    // or subtract from 360
  }

  public void rotateWheel(double speed) {
    for (SwerveModule module : mSwerveModules) {
      module.testRotationMotor(speed);
    }

  }

  public void driveWheel(double speed) {
    for (SwerveModule module : mSwerveModules) {
      module.testDriveMotor(speed);
    }

  }

  public double getGyroRate() {
    return mNavX.getRate();
  }

  public double getRawGyroAngle() {
    double angle = mNavX.getAngle();
    angle %= 360;
    if (angle < 0)
      angle += 360;

    return angle;
  }

  public SwerveModule getSwerveModule(int i) {
    return mSwerveModules[i];
  }

  @Override
  public void holonomicDrive(double forward, double strafe, double rotation, boolean fieldOriented) {
    // for (int i = 0; i < 4; i++) {
    // mSwerveModules[i].testDriveMotor(0.1);
    // mSwerveModules[i].testRotationMotor(0.05);

    // }
    // return;

    forward *= getSpeedMultiplier();
    strafe *= getSpeedMultiplier();

    if (fieldOriented) {
      double angleRad = Math.toRadians(getGyroAngle());
      double temp = forward * Math.cos(angleRad) + strafe * Math.sin(angleRad);
      strafe = -forward * Math.sin(angleRad) + strafe * Math.cos(angleRad);
      forward = temp;
    }

    double a = strafe - rotation * (WHEELBASE / TRACKWIDTH);
    double b = strafe + rotation * (WHEELBASE / TRACKWIDTH);
    double c = forward - rotation * (TRACKWIDTH / WHEELBASE);
    double d = forward + rotation * (TRACKWIDTH / WHEELBASE);

    double[] angles = new double[] { Math.atan2(b, c) * 180 / Math.PI, Math.atan2(b, d) * 180 / Math.PI,
        Math.atan2(a, d) * 180 / Math.PI, Math.atan2(a, c) * 180 / Math.PI };

    double[] speeds = new double[] { Math.sqrt(b * b + c * c), Math.sqrt(b * b + d * d), Math.sqrt(a * a + d * d),
        Math.sqrt(a * a + c * c) };

    double max = speeds[0];

    for (double speed : speeds) {
      if (speed > max) {
        max = speed;
      }
    }

    for (int i = 0; i < 4; i++) {
      if (Math.abs(forward) > 0.05 || Math.abs(strafe) > 0.05 || Math.abs(rotation) > 0.05) {
        mSwerveModules[i].setTargetAngle(angles[i] + 180);
        SmartDashboard.putNumber("Sending Angle" + mSwerveModules[i].getModuleNumber(), angles[i] + 180);
      } else {
        mSwerveModules[i].setTargetAngle(mSwerveModules[i].getTargetAngle());
        SmartDashboard.putNumber("Sending Angle" + mSwerveModules[i].getModuleNumber(), -999);

      }
      mSwerveModules[i].setTargetSpeed(speeds[i]);
    }
  }

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(new cmdHolonomicDrive(this));
    // Make command
  }

  @Override
  public void stopDriveMotors() {
    for (SwerveModule module : mSwerveModules) {
      module.setTargetSpeed(0);
    }
  }

  public void resetMotors() {
    for (int i = 0; i < mSwerveModules.length; i++) {
      mSwerveModules[i].resetMotor();
    }
  }

  public SwerveModule[] getSwerveModules() {
    return mSwerveModules;
  }

  @Override
  public double getMaxAcceleration() {
    return 5.5;
  }

  @Override
  public double getMaxVelocity() {
    return 10;
  }
}
