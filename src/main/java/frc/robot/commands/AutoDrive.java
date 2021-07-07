// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_drive;
  private final LEDSubsystem m_led;

  public AutoDrive(DriveSubsystem drive, LEDSubsystem led) {
      m_led = led;
      m_drive = drive;
      addRequirements(m_drive, m_led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_led.setColor(Constants.LED.red_shot);
      m_drive.arcadeDrive(0.5, 0.0);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
