/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.input;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * Add your docs here.
 */
public final class JoystickX3D extends Joystick {

    private final Button[] mButtons = { new JoystickButton(this, 1), // trigger
            new JoystickButton(this, 2), // side
            new JoystickButton(this, 3), // joystick bottom left
            new JoystickButton(this, 4), // joystick bottom right
            new JoystickButton(this, 5), // joystick top left
            new JoystickButton(this, 6), // joystick top right
            new JoystickButton(this, 7), // base bottom left
            new JoystickButton(this, 8), // base top left
            new JoystickButton(this, 9), // base bottom middle
            new JoystickButton(this, 10), // base top middle
            new JoystickButton(this, 11), // base bottom right
            new JoystickButton(this, 12) // base top right

    };

    public JoystickX3D(int port) {
        super(port);
    }

    public double getYAxis() {
        return getRawAxis(1);

    }

    public double getXAxis() {
        return getRawAxis(0);

    }

    public double getZAxis() {
        return getRawAxis(2);

    }

    public Button getTriggerButton() {
        return mButtons[1];
    }

    public Button getSideButton() {
        return mButtons[2];
    }

    public Button get3Button() {
        return mButtons[3];
    }

    public Button get4Button() {
        return mButtons[4];
    }

    public Button get5Button() {
        return mButtons[5];
    }

    public Button get6Button() {
        return mButtons[6];
    }

    public Button get7Button() {
        return mButtons[7];
    }

    public Button get8Button() {
        return mButtons[8];
    }

    public Button get9Button() {
        return mButtons[9];
    }

    public Button get10Button() {
        return mButtons[10];
    }

    public Button get11Button() {
        return mButtons[11];
    }

    public Button get12Button() {
        return mButtons[12];
    }

}
