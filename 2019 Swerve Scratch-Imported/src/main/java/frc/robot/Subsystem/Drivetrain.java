/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystem;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  private CANSparkMax driveLF, driveRF, driveLR, driveRR;
  private TalonSRX steerLF, steerRF, steerLR, steerRR;

  public static final double WHEEL_BASE_LENGTH = 17; //inches
  public static final double WHEEL_BASE_WIDTH = 17; //inches

  public static final double MAX_SPEED = 0.15; 
  public static final int ENCODER_CPR = 4096; //counts per revolution
  public static final double WHEEL_DIAMETER = 4.0;
  public static final double STEER_DEGREES_PER_COUNT = 360 / ENCODER_CPR; 
  public static final double DRIVE_INCHES_PER_COUNT = (WHEEL_DIAMETER * Math.PI) / (80.0 * 6.67);
  public static final double DEADZONE = 0.08; //variable needs testing
  public static final double MAX_REVERSIBLE_SPEED_DIFFERENCE = 0.5 * MAX_SPEED;

  private static final double DRIVE_P = 7.5, DRIVE_I = 0.0, DRIVE_D = 75.0, DRIVE_F = 1.7;
  private static final double STEER_P = 2.0, STEER_I = 0.0, STEER_D = 20.0;
  private static final int DRIVE_I_ZONE = 0, DRIVE_ALLOWABLE_ERROR = 0, DRIVE_MEASUREMENT_WINDOW = 1;
  public static final VelocityMeasPeriod DRIVE_MEASUREMENT_PERIOD = VelocityMeasPeriod.Period_20Ms; 
  private static final int STATUS_FRAME_PERIOD = 20;
  private static final double RAMP_RATE = 0.5;

  public static final double OMEGA_SCALE = 1.0 / 30.0;
  
  public Drivetrain() {
    driveLF = new CANSparkMax(RobotMap.driveLF, MotorType.kBrushless);
  driveLF.restoreFactoryDefaults();
  driveLF.setInverted(false);
  driveLF.setOpenLoopRampRate(RAMP_RATE);

    driveRF = new CANSparkMax(RobotMap.driveRF, MotorType.kBrushless);
  driveRF.restoreFactoryDefaults();
  driveRF.setInverted(false);
  driveRF.setOpenLoopRampRate(RAMP_RATE);

    driveLR = new CANSparkMax(RobotMap.driveLR, MotorType.kBrushless);
  driveLR.restoreFactoryDefaults();
  driveLR.setInverted(false);
  driveLR.setOpenLoopRampRate(RAMP_RATE);

    driveRR = new CANSparkMax(RobotMap.driveRR, MotorType.kBrushless);
  driveRR.restoreFactoryDefaults();
  driveRR.setInverted(false);
  driveRR.setOpenLoopRampRate(RAMP_RATE);

    steerLF = new TalonSRX(RobotMap.steerLF);
  steerLF.configFactoryDefault();
  steerLF.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
  steerLF.setInverted(true);

    steerRF = new TalonSRX(RobotMap.steerRF);
  steerRF.configFactoryDefault();
  steerRF.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
  steerRF.setInverted(true);

    steerLR = new TalonSRX(RobotMap.steerLR);
  steerLR.configFactoryDefault();
  steerLR.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
  steerLR.setInverted(true);

    steerRR = new TalonSRX(RobotMap.steerRR);
  steerRR.configFactoryDefault();
  steerRR.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
  steerRR.setInverted(true);

    steerLF = new TalonSRX(RobotMap.steerLF);
  steerLF.configFactoryDefault();
  steerLF.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
  steerLF.config_kP(0, STEER_P, 0);
  steerLF.config_kI(0, STEER_I, 0);
  steerLF.config_kD(0, STEER_D, 0);
  steerLF.config_IntegralZone(0, 100, 0);
  steerLF.configAllowableClosedloopError(0, 5, 0);
  steerLF.setNeutralMode(NeutralMode.Brake);
  steerLF.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

    steerRF = new TalonSRX(RobotMap.steerRF);
  steerRF.configFactoryDefault();
  steerRF.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
  steerRF.config_kP(0, STEER_P, 0);
  steerRF.config_kI(0, STEER_I, 0);
  steerRF.config_kD(0, STEER_D, 0);
  steerRF.config_IntegralZone(0, 100, 0);
  steerRF.configAllowableClosedloopError(0, 5, 0);
  steerRF.setNeutralMode(NeutralMode.Brake);
  steerRF.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

    steerLR = new TalonSRX(RobotMap.steerLR);
  steerLR.configFactoryDefault();
  steerLR.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
  steerLR.config_kP(0, STEER_P, 0);
  steerLR.config_kI(0, STEER_I, 0);
  steerLR.config_kD(0, STEER_D, 0);
  steerLR.config_IntegralZone(0, 100, 0);
  steerLR.configAllowableClosedloopError(0, 5, 0);
  steerLR.setNeutralMode(NeutralMode.Brake);
  steerLR.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

    steerRR = new TalonSRX(RobotMap.steerRR);
  steerRR.configFactoryDefault();
  steerRR.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
  steerRR.config_kP(0, STEER_P, 0);
  steerRR.config_kI(0, STEER_I, 0);
  steerRR.config_kD(0, STEER_D, 0);
  steerRR.config_IntegralZone(0, 100, 0);
  steerRR.configAllowableClosedloopError(0, 5, 0);
  steerRR.setNeutralMode(NeutralMode.Brake);
  steerRR.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);
}

  public void swerveDrive(double translationX, double translationY, double rotation) {
    double omegaL2 = rotation * (WHEEL_BASE_LENGTH / 2.0);
    double omegaW2 = rotation * (WHEEL_BASE_WIDTH / 2.0);

    //compute constants used for calculating speeds and angles
    double A = translationX - omegaL2;
    double B = translationX + omegaL2;
    double C = translationY - omegaW2;
    double D = translationY + omegaW2;

    //compute drive motor speeds
    double speedLF = speed(B, D);
    double speedRF = speed(B, C);
    double speedLR = speed(A, D);
    double speedRR = speed(A, C);

    double angleLF = angle(B, D) - 90;
    double angleRF = angle(B, C) - 90;
    double angleLR = angle(A, D) + 90;
    double angleRR = angle(A, C) + 90;

    double maxSpeed = Collections.max(Arrays.asList(speedLF, speedRF, speedLR, speedRR, 1.0));

    setSwerveModule(steerLF, driveLF, angleLF, speedLF / maxSpeed);
    setSwerveModule(steerRF, driveRF, angleRF, speedRF / maxSpeed);
    setSwerveModule(steerLR, driveLR, angleLR, speedLR / maxSpeed);
    setSwerveModule(steerRR, driveRR, angleRR, speedRR / maxSpeed);
  }

  public double speed(double val1, double val2){
    return Math.hypot(val1, val2);
  }

  private double angle(double val1, double val2){
    return Math.toDegrees(Math.atan2(val1, val2));
  }

  private void setSwerveModule(TalonSRX steer, CANSparkMax drive, double angle, double speed){
    double currentPosition = steer.getSelectedSensorPosition(0);
    double currentAngle = (currentPosition * 360.0 / ENCODER_CPR) % 360.0;
    if (currentAngle > 180.0){
      currentAngle -= 360.0;
    }

    double targetAngle = -angle;
    double deltaDegrees = targetAngle - currentAngle;
    if (Math.abs(deltaDegrees) > 180.0){
      deltaDegrees -= 360.0 * Math.signum(deltaDegrees);
    }

    if (Math.abs(deltaDegrees) > 90.0) {
      deltaDegrees -= 180.0 * Math.signum(deltaDegrees);
        speed = -speed;
    }

    if (OI.rightJoy.getAxisType(1)<0.01){
      drive.set(speed=0);
    }
    if (OI.rightJoy.getAxisType(2)<0.01){
      drive.set(speed=0);
    }

    double targetPosition = currentPosition + deltaDegrees * ENCODER_CPR / 360.0;
    steer.set(ControlMode.Position, targetPosition);
    drive.set(speed);
  }

  public double getSteerLFEncoder(){
    return steerLF.getSelectedSensorPosition(0);
  }

  public double getSteerRFEncoder(){
    return steerRF.getSelectedSensorPosition(0);
  }

  public double getSteerLREncoder(){
    return steerLR.getSelectedSensorPosition(0);
  }

  public double getSteerRREncoder(){
    return steerRR.getSelectedSensorPosition(0);
  }

  public void setDriveLF(double speed){
    driveLF.set(speed);
  }

  public void setDriveRF(double speed){
    driveRF.set(speed);
  }

  public void setDriveLR(double speed){
    driveLR.set(speed);
  }

  public void setDriveRR(double speed){
    driveRR.set(speed);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new FieldCentricSwerve());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
