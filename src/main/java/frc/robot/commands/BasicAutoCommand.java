// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class BasicAutoCommand extends SequentialCommandGroup {

    public BasicAutoCommand(DriveSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shooter, LEDSubsystem led) {
        addCommands(new SequentialCommandGroup(
                new ParallelCommandGroup(new RunCommand(() -> shooter.runMotors(2000)),
                        new RunCommand(() -> intake.runFeeder(-Constants.AuxConstants.kFeederMotorSpeed)))
                                .withTimeout(5),
                new ParallelCommandGroup(new AutoDrive(drive, led), new RunCommand(() -> shooter.stopMotors()),
                        new RunCommand(() -> intake.stopFeeder())).withTimeout(3)));
    }
}
