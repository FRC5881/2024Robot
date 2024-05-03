// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DifferentialDrive.ArcadeDrive;
import frc.robot.commands.DifferentialDrive.CurvatureDrive;
import frc.robot.commands.DifferentialDrive.TankDrive;
import frc.robot.commands.SwerveDrive.RobotRelativeDrive;
import frc.robot.commands.SwerveDrive.FieldRelativeAbsoluteAngleDrive;
import frc.robot.commands.SwerveDrive.FieldRelativeRotationRateDrive;
import frc.robot.subsystems.AmpGuideSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Robot.RobotFrame;
import frc.robot.subsystems.DifferentialDriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.DoubleTransformer;
import frc.robot.utils.SendableChooserCommand;

import java.util.Optional;
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
    private Optional<SwerveSubsystem> m_swerveDrive = Optional.empty();
    private Optional<DifferentialDriveSubsystem> m_differentialDrive = Optional.empty();
    @SuppressWarnings("unused")
    private Optional<ClimberSubsystem> m_climber = Optional.empty();
    private Optional<GroundIntakeSubsystem> m_intake = Optional.empty();
    private Optional<AmpGuideSubsystem> m_guide = Optional.empty();

    private Optional<IndexerSubsystem> m_indexer = Optional.empty();
    private Optional<ShooterSubsystem> m_shooter = Optional.empty();

    private final CommandPS5Controller m_driverController = new CommandPS5Controller(
            OperatorConstants.kDriverControllerPort);

    public RobotContainer(RobotFrame bot) {
        // Each bot has a different set of subsystems
        switch (bot) {
            case COMP:
                setupSwerveDrive(bot);
                setupClimber();
                setupShooter();
                setupIndexer();
                setupIntake();
                setupGuide();
                break;
            case M1C2:
                setupSwerveDrive(bot);
                break;
            case DOUGHNUT:
                setupDifferentialDrive();
                break;
        }

        // Swerve drive and Differential drive are mutually exclusive
        if (m_swerveDrive.isPresent() && m_differentialDrive.isPresent()) {
            throw new RuntimeException("Cannot have both swerve and differential drive subsystems");
        }

        if (m_shooter.isPresent() && m_indexer.isPresent()) {
            var shooter = m_shooter.get();
            var indexer = m_indexer.get();

            // By default the "releaseNote" command only runs the indexer
            Supplier<Command> releaseNote = () -> indexer.cSendShooter();

            // Adding the ground intake to the "releaseNote" command is optional
            if (m_intake.isPresent()) {
                var intake = m_intake.get();
                releaseNote = () -> Commands.parallel(indexer.cSendShooter(), intake.cRunLowSpeed());
            }

            final Supplier<Command> releaseNoteFinal = releaseNote;

            // Shoot into the SPEAKER
            Command shootHigh = shooter.cRunWhenSpeakerReady(releaseNoteFinal.get());
            m_driverController.R2().whileTrue(shootHigh);

            // An autonomous command that automatically spins up and shoots a NOTE
            Command autoShootHigh = Commands.race(
                    shooter.cRunWhenSpeakerReady(releaseNoteFinal.get()),
                    Commands.waitSeconds(1.75));

            NamedCommands.registerCommand("SPEAKER", autoShootHigh);

            if (m_guide.isPresent()) {
                var guide = m_guide.get();

                Command shootLow = Commands.parallel(
                        shooter.cRunWhenAmpReady(releaseNoteFinal.get()),
                        guide.cExtend());

                m_driverController.R1().whileTrue(shootLow);
            } else {
                m_driverController.R1().whileTrue(shooter.cRunWhenAmpReady(releaseNoteFinal.get()));
            }

            // Intake from the SOURCE
            m_driverController.L2().whileTrue(shooter.cIntake().alongWith(indexer.cSendDown()));
        }

        // Setup the autonomous chooser
        if (bot == RobotFrame.COMP && m_shooter.isPresent() && m_swerveDrive.isPresent()) {
            autoChooser = AutoBuilder.buildAutoChooser();
            SmartDashboard.putData("Autonomous Command", autoChooser);
        } else {
            autoChooser = null;
        }
    }

    private final SendableChooser<Command> autoChooser;

    public Command getAutonomousCommand() {
        if (autoChooser == null) {
            return Commands.none();
        }

        return autoChooser.getSelected();
    }

    private void setupSwerveDrive(RobotFrame bot) {
        SwerveSubsystem drive = null;

        try {
            drive = new SwerveSubsystem(bot);
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

        // All Drive Commands
        drive.setDefaultCommand(
                new SendableChooserCommand("Swerve Drive Command", rotationRate, robotRelative, absoluteAngle,
                        reversedRobotRelative));

        // Reset gyro
        m_driverController.touchpad().onTrue(drive.cZeroGyro());

        // Lock Pose
        m_driverController.L3().whileTrue(drive.cLock());

        m_swerveDrive = Optional.of(drive);
    }

    private void setupDifferentialDrive() {
        var drive = new DifferentialDriveSubsystem();

        SmartDashboard.putNumber(OperatorConstants.kDriveSensitivity, 1.0);
        SmartDashboard.putNumber(OperatorConstants.kTurnSensitivity, 1.5);

        var leftY = DoubleTransformer.of(m_driverController::getLeftY).negate().deadzone();
        var rightY = DoubleTransformer.of(m_driverController::getRightY).negate().deadzone();
        var rightX = DoubleTransformer.of(m_driverController::getRightX).negate().deadzone();

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
        var intake = new GroundIntakeSubsystem();

        // Intake a note from the ground
        m_driverController.L1().whileTrue(intake.cRunUntilCaptured().repeatedly());
        NamedCommands.registerCommand("INTAKE", intake.cRunUntilCaptured());

        m_intake = Optional.of(intake);
    }

    private void setupShooter() {
        var shooter = new ShooterSubsystem();

        m_driverController.L2().whileTrue(shooter.cIntake());

        m_shooter = Optional.of(shooter);
    }

    private void setupIndexer() {
        var indexer = new IndexerSubsystem();

        m_indexer = Optional.of(indexer);
    }

    private void setupGuide() {
        var guide = new AmpGuideSubsystem();

        m_driverController.circle().onTrue(guide.cExtend());
        m_driverController.cross().onTrue(guide.cRetract());

        m_guide = Optional.of(guide);
    }
}
