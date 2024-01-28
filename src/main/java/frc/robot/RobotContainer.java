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
import frc.robot.utils.PickerCommandFactory;
import frc.robot.utils.DoubleTransformer;

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

        // Sets the shooter to amp mode
        m_driverController.square().onTrue(mode.cSetAmpMode());

        // Sets the shooter to speaker mode
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

        // Some commands might require multiple subsystems
        if (m_shooter.isPresent()) {
            ShooterSubsystem shooter = m_shooter.get();

            if (m_indexer.isPresent()) {
                IndexerSubsystem indexer = m_indexer.get();

                Command shootHigh = shooter.cShootLow().andThen(shooter.cWaitForSetPoint())
                        .andThen(indexer.cSendShooter());
                Command shootLow = shooter.cShootHigh().andThen(shooter.cWaitForSetPoint())
                        .andThen(indexer.cSendShooter());

                Command shoot = Commands.either(
                        shootLow,
                        shootHigh,
                        mode::isAmpMode);

                // Shoot!
                m_driverController.R2().whileTrue(shoot);

                Command sourceIntake = shooter.cSourceIntake().andThen(indexer.cPositionNote());

                // Intake from source
                m_driverController.L2().whileTrue(sourceIntake);

            } else {
                Command shoot = Commands.either(shooter.cShootLow(),
                        shooter.cShootHigh(),
                        mode::isAmpMode);

                // Shoot!
                m_driverController.R2().whileTrue(shoot);

                Command sourceIntake = shooter.cSourceIntake();

                // Intake from source
                m_driverController.L2().whileTrue(sourceIntake);

            }
        }

        if (m_intake.isPresent()) {
            IntakeSubsystem intake = m_intake.get();

            if (m_indexer.isPresent()) {
                IndexerSubsystem indexer = m_indexer.get();
                Command groundIntake = intake.cRun().andThen(indexer.cPositionNote());

                // Intake from source
                m_driverController.R1().whileTrue(groundIntake);
            } else {
                Command groundIntake = intake.cRun();

                // Intake from source
                m_driverController.R1().whileTrue(groundIntake);
            }

        }
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

        // Construct with just a list of commands

        // Uses <T>.getClass().getSimpleName() to name commands
        // Puts the names of the commands up on the smartdashboard as a drop down
    }

    private void setupDifferentialDrive() {
        var subsystem = new DifferentialDriveSubsystem();

        SmartDashboard.putNumber("drive sensitivity", 1.0);
        SmartDashboard.putNumber("turn sensitivity", 1.0);

        var leftY = DoubleTransformer.of(m_driverController::getLeftY)
                .negate()
                .deadzone(0.03);
        // .signedSquare();

        var rightY = DoubleTransformer.of(m_driverController::getRightY)
                .negate()
                .deadzone(0.03);
        // .signedSquare();

        var rightX = DoubleTransformer.of(m_driverController::getRightX)
                .negate()
                .deadzone(0.03);
        // .signedSquare();

        Command arcade = new ArcadeDrive(subsystem, leftY, rightX);
        Command curvature = new CurvatureDrive(subsystem, leftY, rightX, m_driverController.L1());
        Command tank = new TankDrive(subsystem, leftY, rightY);

        PickerCommandFactory chooser = new PickerCommandFactory("Differential Drive Command", arcade, curvature, tank);
        subsystem.setDefaultCommand(chooser.build());

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

        m_shooter = Optional.of(subsystem);
    }

    private void setupIndexer() {
        var subsystem = new IndexerSubsystem();

        // Setup default command

        m_indexer = Optional.of(subsystem);
    }
}
