/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import javax.print.attribute.standard.JobHoldUntil;

import frc.robot.commands.cmdSetEncoder;
import frc.robot.commands.cmdSetFieldOriented;
import frc.robot.input.JoystickX3D;
import frc.robot.input.XboxController;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  public XboxController xboxDriver = new XboxController(1);
  public JoystickX3D joystickDriver = new JoystickX3D(0);

  private Robot mRobot;

  public OI(Robot robot) {
    mRobot = robot;
  }

  public void registerControls() {

    xboxDriver.getLeftBumperButton().whenPressed(new cmdSetFieldOriented(mRobot.getDrivetrain(), false));
    xboxDriver.getLeftBumperButton().whenReleased(new cmdSetFieldOriented(mRobot.getDrivetrain(), true));
    joystickDriver.getTriggerButton().whenPressed(new cmdSetFieldOriented(mRobot.getDrivetrain(), false));
    joystickDriver.getTriggerButton().whenReleased(new cmdSetFieldOriented(mRobot.getDrivetrain(), true));
    joystickDriver.getSideButton().whenPressed(new cmdSetEncoder());

  }

  // public XboxController getXboxController() {
  // return xboxDriver;
  // }

  public JoystickX3D getJoystick() {
    return joystickDriver;
  }

}
