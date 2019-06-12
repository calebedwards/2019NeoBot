/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.input;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Add your docs here.
 */
public final class JoystickX3D extends Joystick {

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

}
