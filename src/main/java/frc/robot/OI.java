/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.cmdDecreaseEncoder;
import frc.robot.commands.cmdDriveWithEncoder;
import frc.robot.commands.cmdIncreaseEncoder;
import frc.robot.input.JoystickX3D;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  public Joystick xboxDriver;
  public JoystickX3D joystickDriver;
  public JoystickButton yButton;
  public JoystickButton aButton;
  public JoystickButton xButton;
  public JoystickButton bButton;

  public OI(NetworkTableInstance tableInstance) {
    xboxDriver = new Joystick(1);
    joystickDriver = new JoystickX3D(0);

    yButton = new JoystickButton(xboxDriver, 4);
    yButton.whenPressed(new cmdDriveWithEncoder());

    aButton = new JoystickButton(xboxDriver, 1);
    aButton.whenPressed(new cmdDriveWithEncoder());

    xButton = new JoystickButton(xboxDriver, 3);
    xButton.whenPressed(new cmdDecreaseEncoder());

    bButton = new JoystickButton(xboxDriver, 2);
    bButton.whenPressed(new cmdIncreaseEncoder());

  }

  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
}
