/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;

/**
 * Add your docs here.
 */
public class SwerveModule extends Subsystem implements PIDOutput {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private final int moduleNumber;
  private final double mZeroOffset;
  private double driveGearRatio = 1;
  private double driveWheelRadius = 2; // find right numbers
  private boolean angleMotorJam = false;
  private long mStallTimeBegin = Long.MAX_VALUE;

  private final CANSparkMax mAngleMotor;
  private final CANSparkMax mDriveMotor;

  public PIDController pidAngle;
  public CANPIDController pidDrive;
  public double d_kP, d_kI, d_kD, d_kIz, d_kFF, d_kMaxOutput, d_kMinOutput, d_kToleranceVolts;
  public double a_kP, a_kI, a_kD, a_kIz, a_kFF, a_kMaxOutput, a_kMinOutput, a_kToleranceVolts;

  private boolean driveInverted = false;
  private double mLastError = 0;
  private double lastTargetAngle = 0;
  double angleSpeed;
  double minVoltage = 0.0;
  double maxVoltage = 5.0;

  public AnalogInput mEncoder;

  public AnalogInput testEncoder = new AnalogInput(RobotMap.encoderFL);

  public SwerveModule(int moduleNumber, int angleMotorID, int driveMotorID, int encoderID, double zeroOffset) {
    this.moduleNumber = moduleNumber;
    this.mZeroOffset = zeroOffset;
    this.mAngleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
    this.mDriveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    pidDrive = mDriveMotor.getPIDController();
    this.mEncoder = new AnalogInput(encoderID);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void driveMotorPIDController() {
    d_kP = 0.0001;
    d_kI = 0;
    d_kD = 0;
    d_kIz = 0;
    d_kFF = 0;
    d_kMaxOutput = 1.0;
    d_kMinOutput = -1.0;

    pidDrive.setP(d_kP);
    pidDrive.setI(d_kI);
    pidDrive.setD(d_kD);
    pidDrive.setIZone(d_kIz);
    pidDrive.setFF(d_kFF);
    pidDrive.setOutputRange(d_kMinOutput, d_kMaxOutput);

    SmartDashboard.putNumber("P - drive", d_kP);
    SmartDashboard.putNumber("I - drive", d_kI);
    SmartDashboard.putNumber("D - drive", d_kD);
    SmartDashboard.putNumber("I - drive", d_kIz);
    SmartDashboard.putNumber("Feed Forward - d", d_kFF);
    SmartDashboard.putNumber("Max Drive Output", d_kMaxOutput);
    SmartDashboard.putNumber("Min Drive Output", d_kMinOutput);
    SmartDashboard.putNumber("Set Inches", 0);
  }

  public void angleMotorPIDController() {
    a_kP = 0.0001;
    a_kI = 0;
    a_kD = 0;
    a_kIz = 0;
    a_kFF = 0;
    a_kMaxOutput = 1.0;
    a_kMinOutput = -1.0;
    a_kToleranceVolts = 0;
    pidAngle = new PIDController(a_kP, a_kI, a_kD, testEncoder, this);
    pidAngle.setInputRange(0, 5.0);
    pidAngle.setOutputRange(-0.5, 1.0);
    pidAngle.setAbsoluteTolerance(a_kToleranceVolts);
    pidAngle.setContinuous(true);
    pidAngle.setOutputRange(a_kMinOutput, a_kMaxOutput);

    SmartDashboard.putNumber("P - angle", a_kP);
    SmartDashboard.putNumber("I - angle", a_kI);
    SmartDashboard.putNumber("D - angle", a_kD);
    SmartDashboard.putNumber("I - angle", a_kIz);
    SmartDashboard.putNumber("Max Angle Output", a_kMaxOutput);
    SmartDashboard.putNumber("Min Angle Output", a_kMinOutput);
    // SmartDashboard.putNumber("Set Angle Inches", 0);
  }

  private double angleToVoltage(double angle) {
    return angle * ((maxVoltage - minVoltage) / 360.0);
  }

  private double voltageToAngle(double voltage) {
    return (voltage * (360.0 / (maxVoltage - minVoltage))) % 360.0;
  }

  public void setDriveInverted(boolean inverted) {
    driveInverted = inverted;
  }

  public void setTargetAngle(double targetAngle) {

    lastTargetAngle = targetAngle;

    targetAngle %= 360;

    SmartDashboard.putNumber("Module Target Angle " + moduleNumber, targetAngle % 360);

    targetAngle += mZeroOffset;

    // double currentAngle = mAngleMotor.getSelectedSensorPosition(0) * (360.0 /
    // 1024.0);
    double currentAngle = voltageToAngle(mEncoder.getAverageVoltage());
    double currentAngleMod = currentAngle % 360;
    if (currentAngleMod < 0)
      currentAngleMod += 360;

    double delta = currentAngleMod - targetAngle;

    if (delta > 180) {
      targetAngle += 360;
    } else if (delta < -180) {
      targetAngle -= 360;
    }

    delta = currentAngleMod - targetAngle;
    if (delta > 90 || delta < -90) {
      if (delta > 90)
        targetAngle += 180;
      else if (delta < -90)
        targetAngle -= 180;
      mDriveMotor.setInverted(false);
    } else {
      mDriveMotor.setInverted(true);
    }

    targetAngle += currentAngle - currentAngleMod;

    // targetAngle *= 1024.0 / 360.0;
    targetAngle = angleToVoltage(targetAngle);

    pidAngle.setSetpoint(targetAngle);
  }

  public void setTargetSpeed(double speed) {
    if (driveInverted) {
      speed = -speed;
    }
    pidDrive.setReference(speed, ControlType.kVelocity);
  }

  public void setDriveGearRatio(double ratio) {
    driveGearRatio = ratio;
  }

  public double getDriveWheelRadius() {
    return driveWheelRadius;
  }

  public void setDriveWheelRadius(double radius) {
    driveWheelRadius = radius;
  }

  public double getTargetAngle() {
    return lastTargetAngle;
  }

  public void resetMotor() {
    angleMotorJam = false;
    mStallTimeBegin = Long.MAX_VALUE;
    SmartDashboard.putBoolean("Motor Jammed" + moduleNumber, angleMotorJam);
  }

  public void setMotionConstraints(double maxAcceleration, double maxVelocity) {
    // need to set max acceleration and max velocity in the sparks
  }

  @Override
  public void pidWrite(double output) {
    angleSpeed = output;
  }

}
