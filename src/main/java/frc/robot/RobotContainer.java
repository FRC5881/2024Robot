// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveDrive.FieldRelativeAbsoluteAngleDrive;
import frc.robot.commands.SwerveDrive.FieldRelativeRotationRateDrive;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.DoubleTransformer;
import frc.robot.utils.SendableChooserCommand;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    // The robot's subsystems are defined here...
    private Optional<SwerveSubsystem> m_swerveDrive = Optional.empty();

    private final CommandXboxController m_driverController = new CommandXboxController(0);

    public RobotContainer() {
        // Each bot has a different set of subsystems
        setupSwerveDrive();

        // Swerve drive and Differential drive are mutually exclusive
        if (m_swerveDrive.isPresent()) {
            throw new RuntimeException("Cannot have both swerve and differential drive subsystems");
        }
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }

    private void setupSwerveDrive() {
        SwerveSubsystem drive = null;

        try {
            drive = new SwerveSubsystem();
        } catch (Exception e) {
            // End the robot program if we can't initialize the swerve drive.
            System.err.println("Failed to initialize swerve drive");
            e.printStackTrace();
            System.exit(1);
        }

        SmartDashboard.putNumber(OperatorConstants.kDriveSensitivity, 1.0);
        SmartDashboard.putNumber(OperatorConstants.kTurnSensitivity, 1.0);

        // Absolute drive commands
        var rightX = DoubleTransformer.of(m_driverController::getRightX);
        var rightY = DoubleTransformer.of(m_driverController::getRightY);

        Supplier<Rotation2d> angle = () -> {
            return new Rotation2d(
                    rightX.deadzone(0.75).getAsDouble(),
                    rightY.deadzone(0.75).getAsDouble());
        };

        var leftX = DoubleTransformer.of(m_driverController::getLeftX).deadzone();
        var leftY = DoubleTransformer.of(m_driverController::getLeftY).deadzone();

        Supplier<Translation2d> translation = () -> {
            return new Translation2d(leftY.getAsDouble(), leftX.getAsDouble());
        };

        Command absoluteAngle = new FieldRelativeAbsoluteAngleDrive(drive, translation, angle);

        // Command absoluteAngleTriangle = new FieldRelativeAbsoluteAngleDrive(drive,
        // translation,
        // Rotation2d.fromDegrees(0));
        // Command absoluteAngleCircle = new FieldRelativeAbsoluteAngleDrive(drive,
        // translation,
        // Rotation2d.fromDegrees(90));
        // Command absoluteAngleSquare = new FieldRelativeAbsoluteAngleDrive(drive,
        // translation,
        // Rotation2d.fromDegrees(180));
        // Command absoluteAngleCross = new FieldRelativeAbsoluteAngleDrive(drive,
        // translation,
        // Rotation2d.fromDegrees(270));

        // m_driverController.triangle().whileTrue(absoluteAngleTriangle);
        // m_driverController.circle().whileTrue(absoluteAngleCircle);
        // m_driverController.square().whileTrue(absoluteAngleSquare);
        // m_driverController.cross().whileTrue(absoluteAngleCross);

        // Relative Drive commands
        Command rotationRate = new FieldRelativeRotationRateDrive(drive, translation, rightX);

        // Reset gyro
        m_driverController.rightStick().onTrue(drive.cZeroGyro());

        drive.setDefaultCommand(new SendableChooserCommand("Swerve Drive Command", rotationRate, absoluteAngle));
        m_swerveDrive = Optional.of(drive);
    }
}
