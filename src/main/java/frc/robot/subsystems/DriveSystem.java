/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.cmdCurvatureDriveXbox;
import frc.robot.commands.cmdTankDriveXbox;

/**
 * Add your docs here.
 */
public class DriveSystem extends Subsystem {
  public CANSparkMax leftMotor = new CANSparkMax(RobotMap.leftMotor, MotorType.kBrushless);
  public CANSparkMax rightMotor = new CANSparkMax(RobotMap.rightMotor, MotorType.kBrushless);
  public CANEncoder leftEncoder;
  public CANEncoder rightEncoder;
  public CANPIDController leftPidController;
  public CANPIDController rightPidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  double rotateToAngleRate;

  // SpeedControllerGroup leftMotors;
  // SpeedControllerGroup rightMotors;
  public DifferentialDrive driveControl;

  public DriveSystem() {
    leftMotor.setClosedLoopRampRate(0.2);
    rightMotor.setClosedLoopRampRate(0.2);
    leftMotor.setOpenLoopRampRate(0.2);
    rightMotor.setOpenLoopRampRate(0.2);
    // leftMotor.setInverted(true);
    rightMotor.setInverted(true);
    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();
    leftEncoder.setPositionConversionFactor(0.57);
    rightEncoder.setPositionConversionFactor(0.57); // converting native units to
    // inches
    leftPidController = leftMotor.getPIDController();
    rightPidController = rightMotor.getPIDController();
    // frontLeftMotor.setInverted(true);
    // frontRightMotor.setInverted(true);
    // leftMotors = new SpeedControllerGroup(backLeftMotor, frontLeftMotor);
    // rightMotors = new SpeedControllerGroup(backRightMotor, frontRightMotor);
    // driveControl = new DifferentialDrive(leftMotor, rightMotor);
    setUpPIDController();

  }

  public void setUpPIDController() {
    kP = 0.0001;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1.0;
    kMinOutput = -1.0;
    rightPidController.setP(kP);
    rightPidController.setI(kI);
    rightPidController.setD(kD);
    rightPidController.setIZone(kIz);
    rightPidController.setFF(kFF);
    rightPidController.setOutputRange(kMinOutput, kMaxOutput);

    leftPidController.setP(kP);
    leftPidController.setI(kI);
    leftPidController.setD(kD);
    leftPidController.setIZone(kIz);
    leftPidController.setFF(kFF);
    leftPidController.setOutputRange(kMinOutput, kMaxOutput);

    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Inches", 0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new cmdTankDriveXbox());
  }

  public void tankDrive() {

    double left = Robot.oi.xboxDriver.getRawAxis(1);
    double right = -Robot.oi.xboxDriver.getRawAxis(5);
    // driveControl.tankDrive(left, right);
    leftMotor.set(left);
    rightMotor.set(right);
  }

  public void curvatureDrive() {
    double throttle = Robot.oi.xboxDriver.getRawAxis(1);
    double turn = Robot.oi.xboxDriver.getRawAxis(4);
    if (Math.abs(throttle) < 0.1) {
      driveControl.curvatureDrive(0, -turn * 0.6, true);
    } else {
      driveControl.curvatureDrive(throttle, -turn, false);
    }

  }

  public void driveForwardTest() {
    // leftEncoder.getPosition();
    // rightEncoder.getPosition();
    // SmartDashboard.putNumber("LeftEnc",
    // Robot.driveSystem.leftEncoder.getPosition());
    // SmartDashboard.putNumber("RightEnc",
    // Robot.driveSystem.rightEncoder.getPosition());
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Inches", 0);

    if ((p != kP)) {
      rightPidController.setP(p);
      leftPidController.setP(p);
      kP = p;
    }
    if ((i != kI)) {
      rightPidController.setI(i);
      leftPidController.setI(i);
      kI = i;
    }
    if ((d != kD)) {
      rightPidController.setD(d);
      leftPidController.setD(d);
      kD = d;
    }
    if ((iz != kIz)) {
      rightPidController.setIZone(iz);
      leftPidController.setIZone(iz);
      kIz = iz;
    }
    if ((ff != kFF)) {
      rightPidController.setFF(ff);
      leftPidController.setFF(ff);
      kFF = ff;
    }
    if ((max != kMaxOutput) || (min != kMinOutput)) {
      rightPidController.setOutputRange(min, max);
      leftPidController.setOutputRange(min, max);
      kMinOutput = min;
      kMaxOutput = max;
    }

    // rotations = 180;
    rightPidController.setReference(rotations, ControlType.kPosition);
    leftPidController.setReference(rotations, ControlType.kPosition);
    // SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("Left Enc Position", leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Enc Position", rightEncoder.getPosition());

  }

  public boolean isAtPosition() {
    double right = rightEncoder.getPosition();
    double left = leftEncoder.getPosition();
    double tolerance = 2; // within two inches
    if (right > 180 - tolerance && right < 180 + tolerance) {
      if (left > 180 - tolerance && left < 180 + tolerance) {
        return true;
      }
    }
    return false;

  }

  public void rotate(double speed) {
    driveControl.curvatureDrive(0, speed, true);
  }

  public void rotateToAngle(double angle) {
    rotate(-rotateToAngleRate);
  }

  public void stopMotors() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public void driveForward(double speed, double angle) {
    driveControl.curvatureDrive(-speed, -rotateToAngleRate, true);
  }

  public void encoderPosition() {
    SmartDashboard.putNumber("Left Enc Position", leftEncoder.getPosition() * -1);
    SmartDashboard.putNumber("Right Enc Position", rightEncoder.getPosition());

  }

}
