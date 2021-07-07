// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ClimberSubsystem extends SubsystemBase {

    // TalonSRX climberMotor = new
    // TalonSRX(Constants.AuxConstants.kClimberMotorPort);
    CANSparkMax climberMotor;
    DoubleSolenoid climbPiston;
    AnalogInput pressureSensor;

    public ClimberSubsystem() {
        climberMotor = new CANSparkMax(Constants.AuxConstants.kClimberMotorPort, MotorType.kBrushless);
        // climberMotor.configFactoryDefault();
        climbPiston = new DoubleSolenoid(Constants.PneumaticConstants.kPistonForwardPort,
                Constants.PneumaticConstants.kPistonReversePort);
        pressureSensor = new AnalogInput(Constants.AuxConstants.kPressureSensorPort);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("PSI", getAirPressure());

    }

    public void runClimber(double speed) {
        // climberMotor.set(ControlMode.PercentOutput, speed);
        climberMotor.set(speed);
    }

    public void stopClimber() {
        // climberMotor.set(ControlMode.PercentOutput, 0);
        climberMotor.set(0);
    }

    public void liftClimber() {
        climbPiston.set(Value.kReverse);
    }

    public void lowerClimber() {
        climbPiston.set(Value.kForward);
    }

    public double getAirPressure() {
        return 250.0 * pressureSensor.getVoltage() / 5.0 - 25.0;
    }
}
