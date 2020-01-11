/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SPI;

/**
 * Add your docs here.
 */
public class RobotMap {

    public static AHRS navX;

    public static final int leftJoystick = 0;
    public static final int rightJoystick = 1;
    public static final int controller = 2;
    
    public static final int driveRF = 11;
    public static final int driveLF = 12;
    public static final int driveLR = 13;
    public static final int driveRR = 14;
    public static final int steerLF = 15;
    public static final int steerRF = 16;
    public static final int steerLR = 17;
    public static final int steerRR = 18;

    public static void init() {
        navX = new AHRS(SPI.Port.kMXP);
    }
}
