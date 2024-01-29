// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.RobotMode;
import frc.robot.commands.DifferentialDrive.ArcadeDrive;
import frc.robot.commands.DifferentialDrive.CurvatureDrive;
import frc.robot.commands.DifferentialDrive.TankDrive;
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

import java.util.Optional;

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

    private RobotMode mode = new RobotMode();

    private final CommandPS5Controller m_driverController = new CommandPS5Controller(
            OperatorConstants.kDriverControllerPort);

    public RobotContainer(RobotFrame bot) {
        // Sets the robot to AmpMode
        m_driverController.square().onTrue(mode.cSetAmpMode());
        // Sets the robot to SpeakerMode
        m_driverController.triangle().onTrue(mode.cSetSpeakerMode());

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

        // "Shoot" commands behave differently if the indexer is present
        //
        // Namely, if we have an indexer it needs to send the NOTE up to the shooter.
        //
        // Without an indexer, the shooter should just run and wait for a human it
        // insert the note
        if (m_shooter.isPresent()) {
            ShooterSubsystem shooter = m_shooter.get();

            Command shoot;
            if (m_indexer.isPresent()) {
                IndexerSubsystem indexer = m_indexer.get();

                Command shootHigh = shooter.cShootLow()
                        .andThen(shooter.cWaitForSetPoint())
                        .andThen(indexer.cSendShooter());
                Command shootLow = shooter.cShootHigh()
                        .andThen(shooter.cWaitForSetPoint())
                        .andThen(indexer.cSendShooter());

                shoot = Commands.either(shootLow, shootHigh, mode::isAmpMode);
            } else {
                shoot = Commands.either(shooter.cShootLow(), shooter.cShootHigh(), mode::isAmpMode);
            }

            m_driverController.R2().whileTrue(shoot);
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
        try {
            var drive = new SwerveSubsystem(visionSubsystem, bot);

            // TODO: Add commands

            m_swerveDrive = Optional.of(drive);
        } catch (Exception e) {
            // End the robot program if we can't initialize the swerve drive.
            System.err.println("Failed to initialize swerve drive");
            e.printStackTrace();
            System.exit(1);
        }
    }

    private void setupDifferentialDrive() {
        var drive = new DifferentialDriveSubsystem();

        SmartDashboard.putNumber("drive sensitivity", 1.0);
        SmartDashboard.putNumber("turn sensitivity", 1.0);

        var leftY = DoubleTransformer.of(m_driverController::getLeftY).negate().deadzone(0.03);
        var rightY = DoubleTransformer.of(m_driverController::getRightY).negate().deadzone(0.03);
        var rightX = DoubleTransformer.of(m_driverController::getRightX).negate().deadzone(0.03);

        Command arcade = new ArcadeDrive(drive, leftY, rightX);
        Command curvature = new CurvatureDrive(drive, leftY, rightX, m_driverController.L1());
        Command tank = new TankDrive(drive, leftY, rightY);

        drive.setDefaultCommand(new SendableChooserCommand("Differential Drive", arcade, curvature, tank));
        m_differentialDrive = Optional.of(drive);
    }

    private void setupClimber() {
        var climber = new ClimberSubsystem();

        // TODO: Add climber commands

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

        m_driverController.L2().whileTrue(shooter.cSourceIntake());

        m_shooter = Optional.of(shooter);
    }

    private void setupIndexer() {
        var indexer = new IndexerSubsystem();

        // The indexer automatically positions NOTES as they are received
        indexer.setDefaultCommand(indexer.cPositionNote());

        m_indexer = Optional.of(indexer);
    }
}
