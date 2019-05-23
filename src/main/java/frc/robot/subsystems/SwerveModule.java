/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
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

  private final CANSparkMax mAngleMotor;
  private final CANSparkMax mDriveMotor;

  public PIDController pidAngle;
  public CANPIDController pidDrive;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, kToleranceVolts;

  private boolean driveInverted = false;
  private double mLastError = 0;
  private double lastTargetAngle = 0;
  double angleSpeed;

  public AnalogInput mEncoder;

  public AnalogInput testEncoder = new AnalogInput(RobotMap.encoderFL);

  public SwerveModule(int moduleNumber, int angleMotorID, int driveMotorID, int encoderID, double zeroOffset) {
    this.moduleNumber = moduleNumber;
    this.mZeroOffset = zeroOffset;
    this.mAngleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
    this.mDriveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    this.mEncoder = new AnalogInput(encoderID);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void driveMotorPIDController() {
    kP = 0.0001;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1.0;
    kMinOutput = -1.0;

    pidDrive.setP(kP);
    pidDrive.setI(kI);
    pidDrive.setD(kD);
    pidDrive.setIZone(kIz);
    pidDrive.setFF(kFF);
    pidDrive.setOutputRange(kMinOutput, kMaxOutput);

    SmartDashboard.putNumber("P - drive", kP);
    SmartDashboard.putNumber("I - drive", kI);
    SmartDashboard.putNumber("D - drive", kD);
    SmartDashboard.putNumber("I - drive", kIz);
    SmartDashboard.putNumber("Feed Forward - d", kFF);
    SmartDashboard.putNumber("Max Drive Output", kMaxOutput);
    SmartDashboard.putNumber("Min Drive Output", kMinOutput);
    SmartDashboard.putNumber("Set Inches", 0);
  }

  public void angleMotorPIDController() {
    kP = 0.0001;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1.0;
    kMinOutput = -1.0;
    kToleranceVolts = 0;
    pidAngle = new PIDController(kP, kI, kD, testEncoder, this);
    pidAngle.setInputRange(0, 5.0);
    pidAngle.setOutputRange(-0.5, 1.0);
    pidAngle.setAbsoluteTolerance(kToleranceVolts);
    pidAngle.setContinuous(true);

    pidAngle.setP(kP);
    pidAngle.setI(kI);
    pidAngle.setD(kD);
    // pidAngle.setIZone(kIz);
    // pidAngle.setFF(kFF);
    pidAngle.setOutputRange(kMinOutput, kMaxOutput);

    SmartDashboard.putNumber("P - angle", kP);
    SmartDashboard.putNumber("I - angle", kI);
    SmartDashboard.putNumber("D - angle", kD);
    SmartDashboard.putNumber("I - angle", kIz);
    SmartDashboard.putNumber("Max Angle Output", kMaxOutput);
    SmartDashboard.putNumber("Min Angle Output", kMinOutput);
    // SmartDashboard.putNumber("Set Angle Inches", 0);
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
    double currentAngle = mEncoder.getAverageVoltage() * (360.0 / 1024.0);
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

    // double currentError = mAngleMotor.getClosedLoopError(0);

    // mLastError = currentError;
    targetAngle *= 1024.0 / 360.0;
    // mAngleMotor.set(ControlMode.Position, targetAngle);
    // figure that one out ^
  }

  public void setTargetSpeed(double speed) {
    if (driveInverted) {
      speed = -speed;
    }
    mDriveMotor.set(speed);
  }

  @Override
  public void pidWrite(double output) {
    angleSpeed = output;
  }

}
