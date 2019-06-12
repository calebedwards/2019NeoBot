/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.HolonomicDriveTrain;

public class cmdSetFieldOriented extends Command {

  private final HolonomicDriveTrain drivetrain;
  private final boolean isFieldOriented;

  @Deprecated
  public cmdSetFieldOriented(HolonomicDriveTrain drivetrain) {
    this(drivetrain, true);
  }

  public cmdSetFieldOriented(HolonomicDriveTrain drivetrain, boolean isFieldOriented) {
    this.drivetrain = drivetrain;
    this.isFieldOriented = isFieldOriented;
  }

  @Override
  protected void execute() {
    drivetrain.setFieldOriented(isFieldOriented);
  }

  @Override
  protected boolean isFinished() {
    return true;
  }
}
