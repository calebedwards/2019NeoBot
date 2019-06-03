/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public abstract class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  /*
   * Abstact keyword is used to create a abstract class and method. Mostly used to
   * provide a base for subclasses to extend and implement the abstract methods
   * and override or use the implemented methods in abstract class. So im using
   * this to reference the length and width in other subsytems.
   */

  private final double width;
  private final double length;
  private double speedMultiplier = 1;

  public DriveTrain(double width, double length) {
    this.width = width;
    this.length = length;
  }

  public final double getWidth() {
    return width;
  }

  public final double getLength() {
    return length;
  }

  public double getSpeedMultiplier() {
    return speedMultiplier;
  }

  public void setSpeedMultiplier(double speedMultiplier) {
    this.speedMultiplier = speedMultiplier;
  }

  public abstract double getMaxAcceleration();

  public abstract double getMaxVelocity();
}
