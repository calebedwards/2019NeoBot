/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.commands.cmdHolonomicDrive;

/**
 * Add your docs here.
 */
public abstract class HolonomicDriveTrain extends DriveTrain {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private double mAdjustmentAngle = 0;
  private boolean mFieldOriented = false;

  public HolonomicDriveTrain(double width, double length) {
    super(width, length);
  }

  public double getAdjustmentAngle() {
    return mAdjustmentAngle;
  }

  public abstract double getGyroAngle();

  public abstract double getRawGyroAngle();

  public void holonomicDrive(double forward, double strafe, double rotation) {
    holonomicDrive(forward, strafe, rotation, isFieldOriented());
  }

  public abstract void holonomicDrive(double forward, double strafe, double rotation, boolean fieldOriented);

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(new cmdHolonomicDrive(this));
    // Make command
  }

  public boolean isFieldOriented() {
    return mFieldOriented;
  }

  public void setAdjustmentAngle(double adjustmentAngle) {
    System.out.printf("New adjustment Angle: % .3f\n", adjustmentAngle);
    mAdjustmentAngle = adjustmentAngle;
  }

  public void setFieldOriented(boolean fieldOriented) {
    mFieldOriented = fieldOriented;
  }

  public abstract void stopDriveMotors();

  public void zeroGyro() {
    setAdjustmentAngle(getRawGyroAngle());
  }
}
