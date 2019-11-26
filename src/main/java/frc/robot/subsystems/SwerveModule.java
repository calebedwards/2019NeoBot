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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedController;

/**
 * Add your docs here.
 */
public class SwerveModule extends Subsystem implements PIDOutput {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public static final boolean enableAngle = true;
  public static final double encoderVolt = 0;

  private final int mModuleNumber;
  private final double mZeroOffset;
  private double driveGearRatio = 4;
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
  double maxVoltage = 4.8;

  public AnalogInput mEncoder;

  // public AnalogInput testEncoder = new AnalogInput(RobotMap.encoderFL);

  public SwerveModule(int moduleNumber, int angleMotorID, int driveMotorID, int encoderID, double zeroOffset) {
    mModuleNumber = moduleNumber;
    mZeroOffset = voltageToAngle(zeroOffset);
    mAngleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
    mDriveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    mEncoder = new AnalogInput(encoderID);
    pidDrive = mDriveMotor.getPIDController();
    angleMotorPIDController();

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
    d_kMaxOutput = 0.5;
    d_kMinOutput = -0.5;

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
    a_kP = 0.5;// 0.07
    a_kI = 0.0;
    a_kD = 0.0001;// 0.08
    a_kIz = 0;
    a_kFF = 0;
    a_kMaxOutput = 0.5;
    a_kMinOutput = -0.5;
    a_kToleranceVolts = 0.01; // 5%
    pidAngle = new PIDController(a_kP, a_kI, a_kD, mEncoder, this);
    maxVoltage = RobotController.getVoltage5V();
    pidAngle.setInputRange(minVoltage, maxVoltage); // ddebug set to min and max
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
    pidAngle.enable();
    pidAngle.setSetpoint(2.0);
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

  public double getEncoderVoltage() {

    return mEncoder.getVoltage();
  }

  public void setTargetAngle(double targetAngle) {

    lastTargetAngle = targetAngle;

    SmartDashboard.putNumber("Module Target Angle " + mModuleNumber, targetAngle % 360);
    SmartDashboard.putNumber("EncoderVoltage" + mModuleNumber, mEncoder.getVoltage());

    targetAngle += mZeroOffset; // ddebug
    targetAngle %= 360;

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
    SmartDashboard.putNumber("targetAngle" + mModuleNumber, targetAngle);

    // targetAngle *= 1024.0 / 360.0;
    targetAngle = angleToVoltage(targetAngle);
    SmartDashboard.putNumber("targetVoltage" + mModuleNumber, targetAngle);

    pidAngle.setSetpoint(targetAngle); // ddebug set back to targetAngle
  }

  public void setTargetSpeed(double speed) {
    if (driveInverted) {
      speed = -speed;
    }
    pidDrive.setReference(speed, ControlType.kDutyCycle);
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
    SmartDashboard.putBoolean("Motor Jammed" + mModuleNumber, angleMotorJam);
  }

  public void setMotionConstraints(double maxAcceleration, double maxVelocity) {
    // need to set max acceleration and max velocity in the sparks
  }

  public void testDriveMotor(double speed) {
    mDriveMotor.set(speed);
  }

  public void testRotationMotor(double speed) {
    mAngleMotor.set(speed);
  }

  public int getModuleNumber() {
    return mModuleNumber;
  }

  @Override
  public void pidWrite(double output) {

    if (!enableAngle) {
      return;
    }
    if (!pidAngle.onTarget() && enableAngle) {
      mAngleMotor.set(-output);
    } else {
      mAngleMotor.set(0);
    }

    SmartDashboard.putNumber("Output" + mModuleNumber, output);
  }

}
