// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveDrive.RobotRelativeDrive;
import frc.robot.commands.SwerveDrive.TwoPlayerDriveCommand;
import frc.robot.commands.SwerveDrive.FieldRelativeAbsoluteAngleDrive;
import frc.robot.commands.SwerveDrive.FieldRelativeRotationRateDrive;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.StraightShooter.StraightShooterSubsystem;
import frc.robot.utils.DoubleTransformer;
import frc.robot.utils.PenningtonLEDs;
import frc.robot.utils.SendableChooserCommand;
import frc.robot.utils.PenningtonLEDs.RawPattern;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class RobotContainer {
    // The robot's subsystems are defined here...
    private SwerveSubsystem drive;
    private ClimberSubsystem climber = new ClimberSubsystem();
    private IntakeSubsystem intake = new IntakeSubsystem();
    private IndexerSubsystem indexer = new IndexerSubsystem();
    private StraightShooterSubsystem shooter = new StraightShooterSubsystem();
    private final PenningtonLEDs leds = new PenningtonLEDs(0, intake);

    private final CommandPS5Controller m_driverController = new CommandPS5Controller(
            OperatorConstants.kDriverControllerPort);
    private final CommandPS5Controller m_copilotController = new CommandPS5Controller(
            OperatorConstants.kCopilotControllerPort);

    private final SendableChooser<Command> autoChooser;

    public boolean hasNote() {
        return intake.hasNote();
    }

    public RobotContainer() {
        setupSwerveDrive();

        // Setup the autonomous chooser
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Autonomous Command", autoChooser);

        // Climber
        m_driverController.povUp().or(m_copilotController.povUp()).whileTrue(climber.cExtend());
        m_driverController.povDown().or(m_copilotController.povDown()).whileTrue(climber.cRetract());

        // Intake
        m_driverController.L1().or(m_copilotController.L1()).whileTrue(
                intake.cRunUntilCaptured().alongWith(leds.cSetPattern(RawPattern.FAST_FLASH_GREEN)));

        NamedCommands.registerCommand("INTAKE", intake.cRunUntilCaptured());

        Supplier<Command> releaseNote = () -> Commands.parallel(indexer.cSendShooter(), intake.cRunLowSpeed());

        // Shoot into the SPEAKER
        Command shootHigh = shooter.cRunAt(12).alongWith(
                Commands.waitSeconds(0.5).andThen(releaseNote.get()))
                .alongWith(leds.cChasingUp());

        m_driverController.R2().or(m_copilotController.R2()).whileTrue(shootHigh);
        NamedCommands.registerCommand("SPEAKER", shootHigh);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void setupSwerveDrive() {
        try {
            drive = new SwerveSubsystem();
        } catch (Exception e) {
            // End the robot program if we can't initialize the swerve drive.
            System.err.println("Failed to initialize swerve drive");
            e.printStackTrace();
            System.exit(1);
        }

        SmartDashboard.putNumber(OperatorConstants.kDriveSensitivity, 1.0);
        SmartDashboard.putNumber(OperatorConstants.kTurnSensitivity, 1.5);
        SmartDashboard.putNumber(OperatorConstants.kAutoTurn, 3.0);

        // Absolute drive commands
        var rightX = DoubleTransformer.of(m_driverController::getRightX).negate();
        var rightY = DoubleTransformer.of(m_driverController::getRightY).negate();

        Supplier<Rotation2d> angle = () -> {
            return new Rotation2d(
                    rightX.deadzone(0.75).getAsDouble(),
                    rightY.deadzone(0.75).getAsDouble());
        };

        var leftX = DoubleTransformer.of(m_driverController::getLeftX).negate().deadzone();
        var leftY = DoubleTransformer.of(m_driverController::getLeftY).negate().deadzone();

        Supplier<Translation2d> translation = () -> {
            return new Translation2d(leftY.getAsDouble(), leftX.getAsDouble());
        };

        Command absoluteAngle = new FieldRelativeAbsoluteAngleDrive(drive, translation, angle);

        // Relative Drive commands
        Command rotationRate = new FieldRelativeRotationRateDrive(drive, translation, rightX);

        // Robot Relative Drive
        Command robotRelative = new RobotRelativeDrive(drive, translation, rightX);
        Command reversedRobotRelative = new RobotRelativeDrive(drive, () -> translation.get().unaryMinus(),
                rightX.negate());
        reversedRobotRelative.setName("ReverseRobotRelative");

        Command twoPlayerDrive = new TwoPlayerDriveCommand(drive, m_driverController, m_copilotController);

        // All Drive Commands
        drive.setDefaultCommand(
                new SendableChooserCommand("Swerve Drive Command", twoPlayerDrive, rotationRate, robotRelative,
                        absoluteAngle,
                        reversedRobotRelative));

        // Reset gyro
        m_driverController.touchpad().onTrue(drive.cZeroGyro());

        // Lock Pose
        m_driverController.L3().or(m_copilotController.L3()).whileTrue(drive.cLock());
    }
}
