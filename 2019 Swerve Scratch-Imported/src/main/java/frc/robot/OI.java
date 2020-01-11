/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Add your docs here.
 */
public class OI {
    public Joystick leftJoy;
    public Joystick rightJoy;
    public XboxController controller; 

    public OI(){
        leftJoy = new Joystick(RobotMap.leftJoystick);
        rightJoy = new Joystick(RobotMap.leftJoystick);
        controller = new XboxController(RobotMap.controller);
    }

    public boolean getAButtonPress(){
        return controller.getAButtonPressed();
    }

    public boolean getAButtonRelease(){
        return controller.getAButtonReleased();
    }

    public boolean getBButtonPress(){
        return controller.getAButtonPressed();
    }

    public boolean getBButton(){
        return controller.getBButton();
    }

    public boolean getBButtonRelease(){
        return controller.getBButtonReleased();
    }

    public boolean getLeftJoyButton(int buttonNumber){
        return leftJoy.getRawButton(buttonNumber);
    }
}
