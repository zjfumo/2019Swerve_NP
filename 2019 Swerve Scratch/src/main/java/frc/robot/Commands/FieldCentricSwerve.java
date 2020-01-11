/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class FieldCentricSwerve extends Command {
  
  public static final double OMEGA_SCALE = 1.0 / 30.0;
  public static final double DEADZONE = 0.05;

  private double originHeading = 0.0;
  private double originCorr = 0;
  private double leftPow = 1.0;
  private double rightPow = 1.0;
  
  public FieldCentricSwerve() {
    requires(Robot.drivetrain);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    originHeading = Robot.zeroHeading;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Robot.oi.getLeftJoyButton(7)){
      //originHeading = RobotMap.navX.getFusedHeading();
    }

    double originOffset = 360 - originHeading;
    //originCorr = RobotMap.navX.getFusedHeading() + originOffset;

    double translationX = Math.pow(Math.abs(Robot.oi.leftJoy.getX()), leftPow) * Math.signum(Robot.oi.leftJoy.getX());
    double translationY = Math.pow(Math.abs(Robot.oi.leftJoy.getY()), leftPow) * -Math.signum(Robot.oi.leftJoy.getY());
    double rotation = Math.pow(Math.abs(Robot.oi.leftJoy.getX()), leftPow) * Math.signum(Robot.oi.leftJoy.getX()) * OMEGA_SCALE;
    // Make this return true when this Command no longer needs to run execute()
  
    if (Math.abs(translationX) < Math.pow(DEADZONE, leftPow)) translationX = 0.0;
    if (Math.abs(translationY) < Math.pow(DEADZONE, leftPow)) translationY = 0.0;
    if (Math.abs(rotation) < Math.pow(DEADZONE, rightPow) * OMEGA_SCALE) rotation = 0.0;
  
    if (translationX == 0.0 && translationY == 0.0 && rotation == 0.0){
      Robot.drive.setDriveLF(0.0);
      Robot.drive.setDriveRF(0.0);
      Robot.drive.setDriveLR(0.0);
      Robot.drive.setDriveRR(0.0);
      return;
    }

    if (!Robot.oi.leftJoy.getTrigger()){
      //double originCorrection = Math.toRadians(originHeading - RobotMap.navX.getFusedHeading());
      double temp = translationY * Math.cos(originCorrection) - translationX * Math.sin(originCorrection);
      translationX = translationX * Math.cos(originCorrection) + translationY * Math.sin(originCorrection);
      translationY = temp;
    }
    
    Robot.drive.swerveDrive(translationX, translationY, rotation);
  }

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
