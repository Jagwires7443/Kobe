// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.JoystickButtons;
// import frc.robot.Constants.LED;

import frc.robot.commands.BasicAutoCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.subsystems.VisionSubsystem;

import java.util.Arrays;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
// @SuppressWarnings("unused")
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  // private final VisionSubsystem m_vision = new VisionSubsystem();
  private final Compressor m_compressor = new Compressor(Constants.PneumaticConstants.pcmPort);
  private final LEDSubsystem m_5vled = new LEDSubsystem(Constants.LED.k5vLEDPWMPort);
  // private final LEDSubsystem m_12vled = new
  // LEDSubsystem(Constants.LED.k12vLEDPWMPort);
  Command BasicAutoCommand = new BasicAutoCommand(m_robotDrive, m_intake, m_shooter, m_5vled);
  Joystick m_driverController = new Joystick(JoystickButtons.kDriverControllerPort);

  double shooterRPM = 2000;

  public Command complexCommand() {
    TrajectoryConfig config = new TrajectoryConfig(Constants.DriveConstants.kMaxVelocityMeters,
        Constants.DriveConstants.kMaxAccelerationMeters).setKinematics(m_robotDrive.getKinematics());
    Trajectory trajectory = TrajectoryGenerator
        .generateTrajectory(Arrays.asList(new Pose2d(), new Pose2d(1.0, 0, new Rotation2d())), config);

    RamseteCommand command = new RamseteCommand(trajectory, m_robotDrive::getPose, new RamseteController(2.0, 0.7),
        m_robotDrive.getFeedForward(), m_robotDrive.getKinematics(), m_robotDrive::getSpeeds,
        m_robotDrive.getLeftPIDController(), m_robotDrive.getRightPIDController(), m_robotDrive::setOutput,
        m_robotDrive);

    return command;
  }

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_compressor.start();
    m_robotDrive.setDefaultCommand(new RunCommand(
        () -> m_robotDrive.arcadeDrive(m_driverController.getRawAxis(Constants.JoystickButtons.kForwardAxis),
            m_driverController.getRawAxis(Constants.JoystickButtons.kTurnAxis)),
        m_robotDrive));

    m_chooser.setDefaultOption("Basic Auto", BasicAutoCommand);
    Shuffleboard.getTab("Auto").add(m_chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(m_driverController, Constants.JoystickButtons.kShooterButton)
        .whileHeld(() -> m_shooter.runMotors(shooterRPM)).whenReleased(() -> m_shooter.stopMotors());

    new JoystickButton(m_driverController, Constants.JoystickButtons.kIntakeButton)
        .whileHeld(() -> m_intake.runIntake(-Constants.AuxConstants.kIntakeMotorSpeed), m_intake)
        .whenReleased(() -> m_intake.stopIntake());

    new JoystickButton(m_driverController, Constants.JoystickButtons.kReverseIntakeButton)
        .whileHeld(() -> m_intake.runIntake(Constants.AuxConstants.kIntakeMotorSpeed), m_intake)
        .whenReleased(() -> m_intake.stopIntake());

    new JoystickButton(m_driverController, Constants.JoystickButtons.kTurboButton)
        .whenPressed(() -> m_robotDrive.setSpeed(0.99)).whenPressed(() -> shooterRPM = 4000);

    new JoystickButton(m_driverController, Constants.JoystickButtons.kFullSpeedButton)
        .whenPressed(() -> m_robotDrive.setSpeed(0.75)).whenPressed(() -> shooterRPM = 1700);

    new JoystickButton(m_driverController, Constants.JoystickButtons.kSlowSpeedButton)
        .whenPressed(() -> m_robotDrive.setSpeed(0.5)).whenPressed(() -> shooterRPM = 2000);

    new JoystickButton(m_driverController, Constants.JoystickButtons.kFeederUpButton)
        .whenPressed(() -> m_intake.runFeeder(Constants.AuxConstants.kFeederMotorSpeed), m_intake)
        .whenReleased(() -> m_intake.stopFeeder());

    new JoystickButton(m_driverController, Constants.JoystickButtons.kFeederDownButton)
        .whenPressed(() -> m_intake.runFeeder(-Constants.AuxConstants.kFeederMotorSpeed), m_intake)
        .whenReleased(() -> m_intake.stopFeeder());

    new JoystickButton(m_driverController, Constants.JoystickButtons.kElevatorUpButton)
        .whenPressed(() -> m_climber.runClimber(Constants.AuxConstants.kClimberMotorSpeed), m_climber)
        .whenReleased(() -> m_climber.stopClimber());

    new JoystickButton(m_driverController, Constants.JoystickButtons.kElevatorDownButton)
        .whenPressed(() -> m_climber.runClimber(-Constants.AuxConstants.kClimberMotorSpeed), m_climber)
        .whenReleased(() -> m_climber.stopClimber());

    new JoystickButton(m_driverController, Constants.JoystickButtons.kPistonUpButton)
        .whenPressed(() -> m_climber.liftClimber());

    new JoystickButton(m_driverController, Constants.JoystickButtons.kPistonDownButton)
        .whenPressed(() -> m_climber.lowerClimber());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
