// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;

public class DriveSubsystem extends SubsystemBase {

  private CANSparkMax frontLeftMotor = new CANSparkMax(Constants.DriveConstants.kLeftMotorFrontPort,
      MotorType.kBrushless);
  private CANSparkMax backLeftMotor = new CANSparkMax(Constants.DriveConstants.kLeftMotorBackPort,
      MotorType.kBrushless);
  private CANEncoder leftEncoder = frontLeftMotor.getEncoder();

  private CANSparkMax frontRightMotor = new CANSparkMax(Constants.DriveConstants.kRightMotorFrontPort,
      MotorType.kBrushless);
  private CANSparkMax backRightMotor = new CANSparkMax(Constants.DriveConstants.kRightMotorBackPort,
      MotorType.kBrushless);
  private CANEncoder rightEncoder = frontRightMotor.getEncoder();

  AHRS gyro = new AHRS(SPI.Port.kMXP);

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
      Constants.DriveConstants.kWidthBetweenWheelsMeters);
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  DifferentialDrive m_drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.397, 0.0775, 0.00973);

  PIDController leftPidController = new PIDController(Constants.DriveConstants.kP, 0.0, 0.0);
  PIDController rightPidController = new PIDController(Constants.DriveConstants.kP, 0.0, 0.0);

  Pose2d pose;
  double m_speed = 0.99;

  public DriveSubsystem() {

    frontLeftMotor.restoreFactoryDefaults();
    frontRightMotor.restoreFactoryDefaults();
    backLeftMotor.restoreFactoryDefaults();
    backRightMotor.restoreFactoryDefaults();

    frontLeftMotor.setClosedLoopRampRate(0.75);
    frontRightMotor.setClosedLoopRampRate(0.75);
    backLeftMotor.setClosedLoopRampRate(0.75);
    backRightMotor.setClosedLoopRampRate(0.75);

    backLeftMotor.follow(frontLeftMotor);
    backRightMotor.follow(frontRightMotor);

    m_drive.setSafetyEnabled(false);
  }

  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive((-fwd * m_speed), (rot * m_speed));
  }

  public void setOutput(double leftVolts, double rightVolts) {
    frontLeftMotor.set(leftVolts / 12);
    frontRightMotor.set(rightVolts / 12);
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public double getTurnRate() {
    return gyro.getRate();
  }

  public Pose2d getPose() {
    return pose;
  }

  public SimpleMotorFeedforward getFeedForward() {
    return feedforward;
  }

  public PIDController getRightPIDController() {
    return rightPidController;
  }

  public PIDController getLeftPIDController() {
    return leftPidController;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        frontLeftMotor.getEncoder().getVelocity() / 10.75 * 2 * Math.PI
            * (Constants.DriveConstants.kWheelDiameterInches / 2) / 60,
        frontRightMotor.getEncoder().getVelocity() / 10.75 * 2 * Math.PI
            * (Constants.DriveConstants.kWheelDiameterInches / 2) / 60);
  }

  public double getLeftWheelSpeed() {
    return frontLeftMotor.getEncoder().getVelocity() / 10.75 * 2 * Math.PI
        * (Constants.DriveConstants.kWheelDiameterInches / 2) / 60;
  }

  public double getRightWheelSpeed() {
    return frontRightMotor.getEncoder().getVelocity() / 10.75 * 2 * Math.PI
        * (Constants.DriveConstants.kWheelDiameterInches / 2) / 60;
  }

  @Override
  public void periodic() {
    pose = odometry.update(getHeading(), getLeftWheelSpeed(), getLeftWheelSpeed());
  }

  public void setSpeed(double speed) {
    m_speed = speed;
  }

  public void resetEncoders() {
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }
}
