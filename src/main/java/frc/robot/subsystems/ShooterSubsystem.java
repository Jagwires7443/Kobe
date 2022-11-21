// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class ShooterSubsystem extends SubsystemBase {

    private final CANSparkMax leftShooterMotor, rightShooterMotor;
    private RelativeEncoder leftShooterEncoder, rightShooterEncoder;
    private SparkMaxPIDController leftPIDController, rightPIDController;
    public double setPoint;

    public ShooterSubsystem() {
        leftShooterMotor = new CANSparkMax(ShooterConstants.kLeftShooterPort, MotorType.kBrushless);
        leftShooterMotor.restoreFactoryDefaults();
        leftShooterMotor.setIdleMode(IdleMode.kCoast);
        leftShooterMotor.setSmartCurrentLimit(60);
        leftShooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
        leftShooterEncoder = leftShooterMotor.getEncoder();
        leftPIDController = leftShooterMotor.getPIDController();
        leftPIDController.setP(Constants.ShooterConstants.kleftP);
        leftPIDController.setI(Constants.ShooterConstants.kleftI);
        leftPIDController.setD(Constants.ShooterConstants.kleftD);
        leftPIDController.setIZone(Constants.ShooterConstants.kleftIz);
        leftPIDController.setFF(Constants.ShooterConstants.kleftFF);
        leftPIDController.setOutputRange(Constants.ShooterConstants.kleftMinOut,
                Constants.ShooterConstants.kleftMaxOut);

        rightShooterMotor = new CANSparkMax(ShooterConstants.kRightShooterPort, MotorType.kBrushless);
        rightShooterMotor.restoreFactoryDefaults();
        rightShooterMotor.setInverted(true);
        rightShooterMotor.setIdleMode(IdleMode.kCoast);
        rightShooterMotor.setSmartCurrentLimit(60);
        rightShooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
        rightShooterEncoder = rightShooterMotor.getEncoder();
        rightPIDController = rightShooterMotor.getPIDController();
        rightPIDController.setP(Constants.ShooterConstants.krightP);
        rightPIDController.setI(Constants.ShooterConstants.krightI);
        rightPIDController.setD(Constants.ShooterConstants.krightD);
        rightPIDController.setIZone(Constants.ShooterConstants.krightIz);
        rightPIDController.setFF(Constants.ShooterConstants.krightFF);
        rightPIDController.setOutputRange(Constants.ShooterConstants.krightMinOut,
                Constants.ShooterConstants.krightMaxOut);

        SmartDashboard.putNumber("Left P", leftPIDController.getP());
        SmartDashboard.putNumber("Right P", rightPIDController.getP());
    }

    public void runMotors(double desiredRPM) {

        leftPIDController.setReference(desiredRPM, ControlType.kVelocity);
        rightPIDController.setReference(desiredRPM, ControlType.kVelocity);

        SmartDashboard.putNumber("Setpoint", desiredRPM);
        SmartDashboard.putNumber("Left Output Current", leftShooterMotor.getOutputCurrent());
        SmartDashboard.putNumber("Right Output Current", rightShooterMotor.getOutputCurrent());
    }

    public void stopMotors() {
        leftShooterMotor.set(0.0);
        rightShooterMotor.set(0.0);
    }

    public void periodic() {
        SmartDashboard.putNumber("Left Velocity", leftShooterEncoder.getVelocity());
        SmartDashboard.putNumber("Right Velocity", rightShooterEncoder.getVelocity());
    }

    public boolean atSpeed() {
        if (((setPoint - leftShooterEncoder.getVelocity()) <= 250)
                && ((setPoint - rightShooterEncoder.getVelocity()) <= 250)) {
            return true;
        } else {
            return false;
        }
    }
}
