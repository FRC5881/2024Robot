// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.RobotMode;
import frc.robot.commands.DifferentialDrive.ArcadeDrive;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Robot.RobotFrame;
import frc.robot.subsystems.DifferentialDriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class RobotContainer {
    // The robot's subsystems are defined here...
    private Optional<VisionSubsystem> m_vision = Optional.empty();
    private Optional<SwerveSubsystem> m_swerveDrive = Optional.empty();
    private Optional<DifferentialDriveSubsystem> m_differentialDrive = Optional.empty();
    private Optional<ClimberSubsystem> m_climber = Optional.empty();
    private Optional<IntakeSubsystem> m_intake = Optional.empty();
    private Optional<ShooterSubsystem> m_shooter = Optional.empty();

    private final CommandPS5Controller m_driverController = new CommandPS5Controller(
            OperatorConstants.kDriverControllerPort);

    public RobotContainer(RobotFrame bot) {
        // Each bot has a different set of subsystems
        switch (bot) {
            case COMP:
                // TODO: Build the COMP bot
                break;
            case M1C2:
                // TODO: Vision?
                setupSwerveDrive(m_vision, bot);
                break;
            case DOUGHNUT:
                setupDifferentialDrive();
                setupShooter();
                break;
        }

        // Swerve drive and Differential drive are mutually exclusive
        if (m_swerveDrive.isPresent() && m_differentialDrive.isPresent()) {
            throw new RuntimeException("Cannot have both swerve and differential drive subsystems");
        }

        // Some commands might require multiple subsystems
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }

    private void setupVision() {
        var subsystem = new VisionSubsystem();

        m_vision = Optional.of(subsystem);
    }

    private void setupSwerveDrive(Optional<VisionSubsystem> visionSubsystem, RobotFrame bot) {
        try {
            var subsystem = new SwerveSubsystem(visionSubsystem, bot);
            m_swerveDrive = Optional.of(subsystem);
        } catch (Exception e) {
            // End the robot program if we can't initialize the swerve drive.
            System.err.println("Failed to initialize swerve drive");
            e.printStackTrace();
            System.exit(1);
        }
    }

    private void setupDifferentialDrive() {
        var subsystem = new DifferentialDriveSubsystem();

        Command teleopDriveCommand = new ArcadeDrive(subsystem,
                signedSquare(deadzone(negate(m_driverController::getLeftY), 0.05)),
                signedSquare(deadzone(negate(m_driverController::getRightX), 0.05)));

        subsystem.setDefaultCommand(teleopDriveCommand);

        m_differentialDrive = Optional.of(subsystem);
    }

    private void setupClimber() {
        var subsystem = new ClimberSubsystem();

        // TODO: Add climber commands

        m_climber = Optional.of(subsystem);
    }

    private void setupIntake() {
        var subsystem = new IntakeSubsystem();

        // TODO: Add intake commands

        m_intake = Optional.of(subsystem);
    }

    private void setupShooter() {
        var subsystem = new ShooterSubsystem();

        // Sets the mode of the shooter
        RobotMode mode = new RobotMode();
        Command shoot = Commands.either(subsystem.cShootLow(),
                subsystem.cShootHigh(),
                mode::isAmpMode);

        // Shoot!
        m_driverController.R1().whileTrue(shoot);

        // Sets the shooter to amp mode
        m_driverController.square().onTrue(mode.cSetAmpMode());

        // Sets the shooter to speaker mode
        m_driverController.triangle().onTrue(mode.cSetSpeakerMode());

        // sets the shooter to intake from a human player
        m_driverController.L1().whileTrue(subsystem.cSourceIntake());

        m_shooter = Optional.of(subsystem);
    }

    private static DoubleSupplier deadzone(DoubleSupplier supplier, double deadzone) {
        return () -> {
            double value = supplier.getAsDouble();
            if (Math.abs(value) < deadzone) {
                return 0.0;
            }
            return value;
        };
    }

    private static DoubleSupplier negate(DoubleSupplier supplier) {
        return () -> {
            return -supplier.getAsDouble();
        };
    }

    private static DoubleSupplier signedSquare(DoubleSupplier supplier) {
        return () -> {
            double x = supplier.getAsDouble();
            return x < 0 ? -x * x : x * x;
        };
    }
}
