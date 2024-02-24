// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.DifferentialDrive.ArcadeDrive;
import frc.robot.commands.DifferentialDrive.CurvatureDrive;
import frc.robot.commands.DifferentialDrive.TankDrive;
import frc.robot.commands.SwerveDrive.FieldRelativeAbsoluteAngleDrive;
import frc.robot.commands.SwerveDrive.FieldRelativeRotationRateDrive;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Robot.RobotFrame;
import frc.robot.subsystems.DifferentialDriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.DoubleTransformer;
import frc.robot.utils.SendableChooserCommand;

import static edu.wpi.first.units.Units.Percent;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private Optional<IndexerSubsystem> m_indexer = Optional.empty();
    private Optional<ShooterSubsystem> m_shooter = Optional.empty();

    private final CommandPS5Controller m_driverController = new CommandPS5Controller(
            OperatorConstants.kDriverControllerPort);

    public RobotContainer(RobotFrame bot) {
        // Each bot has a different set of subsystems
        switch (bot) {
            case COMP:
                setupSwerveDrive(m_vision, bot);
                setupClimber();
                setupShooter();
                setupIndexer();
                break;
            case M1C2:
                // setupSwerveDrive(m_vision, bot);
                break;
            case DOUGHNUT:
                // setupDifferentialDrive();
                break;
        }

        // Swerve drive and Differential drive are mutually exclusive
        if (m_swerveDrive.isPresent() && m_differentialDrive.isPresent()) {
            throw new RuntimeException("Cannot have both swerve and differential drive subsystems");
        }

        // "Shoot" commands behave differently if the indexer is present
        //
        // Namely, if we have an indexer it needs to send the NOTE up to the shooter.
        //
        // Without an indexer, the shooter should just run and wait for a human it
        // insert the note
        if (m_shooter.isPresent()) {
            ShooterSubsystem shooter = m_shooter.get();

            if (m_indexer.isPresent()) {
                IndexerSubsystem indexer = m_indexer.get();

                Command shootHigh = shooter.cRunWhenAmpReady(indexer.cSendDown());
                Command shootLow = shooter.cRunWhenAmpReady(indexer.cSendShooter());

                m_driverController.R1().whileTrue(shootLow);
                m_driverController.R2().whileTrue(shootHigh);
            } else {
                Command shootHigh = shooter.cPercentOutput(Percent.of(100));
                Command shootLow = shooter.cSetpoint(ShooterConstants.kShooterAmpSpeed);

                m_driverController.R1().whileTrue(shootLow);
                m_driverController.R2().whileTrue(shootHigh);
            }
        }
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }

    private void setupVision() {
        var vision = new VisionSubsystem();

        m_vision = Optional.of(vision);
    }

    private void setupSwerveDrive(Optional<VisionSubsystem> visionSubsystem, RobotFrame bot) {
        SwerveSubsystem drive = null;

        try {
            drive = new SwerveSubsystem(visionSubsystem, bot);
        } catch (Exception e) {
            // End the robot program if we can't initialize the swerve drive.
            System.err.println("Failed to initialize swerve drive");
            e.printStackTrace();
            System.exit(1);
        }

        SmartDashboard.putNumber(OperatorConstants.kDriveSensitivity, 1.0);
        SmartDashboard.putNumber(OperatorConstants.kTurnSensitivity, 1.0);

        // Absolute drive commands
        var rightX = DoubleTransformer.of(m_driverController::getRightX).negate();
        var rightY = DoubleTransformer.of(m_driverController::getRightY).negate();

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
        Command absoluteAngleTriangle = new FieldRelativeAbsoluteAngleDrive(drive, translation,
                Rotation2d.fromDegrees(0));
        Command absoluteAngleCircle = new FieldRelativeAbsoluteAngleDrive(drive, translation,
                Rotation2d.fromDegrees(90));
        Command absoluteAngleSquare = new FieldRelativeAbsoluteAngleDrive(drive, translation,
                Rotation2d.fromDegrees(180));
        Command absoluteAngleCross = new FieldRelativeAbsoluteAngleDrive(drive, translation,
                Rotation2d.fromDegrees(270));

        m_driverController.triangle().whileTrue(absoluteAngleTriangle);
        m_driverController.circle().whileTrue(absoluteAngleCircle);
        m_driverController.square().whileTrue(absoluteAngleSquare);
        m_driverController.cross().whileTrue(absoluteAngleCross);

        // Relative Drive commands
        Command rotationRate = new FieldRelativeRotationRateDrive(drive, translation, rightX);

        // Reset gyro
        m_driverController.touchpad().onTrue(drive.cZeroGyro());

        drive.setDefaultCommand(new SendableChooserCommand("Swerve Drive Command", rotationRate, absoluteAngle));
        m_swerveDrive = Optional.of(drive);
    }

    private void setupDifferentialDrive() {
        var drive = new DifferentialDriveSubsystem();

        SmartDashboard.putNumber(OperatorConstants.kDriveSensitivity, 1.0);
        SmartDashboard.putNumber(OperatorConstants.kTurnSensitivity, 1.0);

        var leftY = DoubleTransformer.of(m_driverController::getLeftY).negate().deadzone(0.03);
        var rightY = DoubleTransformer.of(m_driverController::getRightY).negate().deadzone(0.03);
        var rightX = DoubleTransformer.of(m_driverController::getRightX).negate().deadzone(0.03);

        Command arcade = new ArcadeDrive(drive, leftY, rightX);
        Command curvature = new CurvatureDrive(drive, leftY, rightX, m_driverController.L1());
        Command tank = new TankDrive(drive, leftY, rightY);

        drive.setDefaultCommand(new SendableChooserCommand("Differential Drive Command", arcade, curvature, tank));
        m_differentialDrive = Optional.of(drive);
    }

    private void setupClimber() {
        var climber = new ClimberSubsystem();

        m_driverController.povUp().whileTrue(climber.cExtend());
        m_driverController.povDown().whileTrue(climber.cRetract());

        m_climber = Optional.of(climber);
    }

    private void setupIntake() {
        var intake = new IntakeSubsystem();

        // Intake a note from the ground
        m_driverController.R1().whileTrue(intake.cRun());

        m_intake = Optional.of(intake);
    }

    private void setupShooter() {
        var shooter = new ShooterSubsystem();

        m_driverController.L2().whileTrue(shooter.cIntake());

        m_shooter = Optional.of(shooter);
    }

    private void setupIndexer() {
        var indexer = new IndexerSubsystem();

        // The indexer automatically positions NOTES as they are received
        indexer.setDefaultCommand(indexer.cPositionNote());

        m_driverController.povLeft().whileTrue(indexer.cSendDown());
        m_driverController.povRight().whileTrue(indexer.cSendShooter());

        m_indexer = Optional.of(indexer);
    }
}
