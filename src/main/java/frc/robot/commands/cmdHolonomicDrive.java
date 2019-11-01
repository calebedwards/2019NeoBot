/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.HolonomicDriveTrain;

public class cmdHolonomicDrive extends Command {
  private final HolonomicDriveTrain mDrivetrain;

  public cmdHolonomicDrive(HolonomicDriveTrain drivetrain) {
    mDrivetrain = drivetrain;

    requires(drivetrain);
  }

  private double deadband(double input) {
    if (Math.abs(input) < 0.05)
      return 0;
    return input;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double forward = Robot.getOI().getJoystick().getYAxis();

    double strafe = -Robot.getOI().getJoystick().getXAxis();

    double rotation = Robot.getOI().getJoystick().getZAxis();
    // System.out.printf("F:%f S:%f R:%f", forward, strafe, rotation);

    forward *= Math.abs(forward);
    strafe *= Math.abs(strafe);
    rotation *= Math.abs(rotation);

    forward = deadband(forward);
    strafe = deadband(strafe);
    rotation = deadband(rotation);

    SmartDashboard.putNumber("Forward", forward);
    SmartDashboard.putNumber("Strafe", strafe);
    SmartDashboard.putNumber("Rotation", rotation);

    mDrivetrain.holonomicDrive(forward, strafe, rotation);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}