// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DifferentialDriveCommand;
import frc.robot.commands.Mode;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Robot.Bot;
import frc.robot.subsystems.DifferentialDriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class RobotContainer {
    private VisionSubsystem m_vision;
    private SwerveSubsystem m_swerveDrive;
    private DifferentialDriveSubsystem m_differentialDrive;
    private ClimberSubsystem m_climber;
    private IntakeSubsystem m_intake;
    private ShooterSubsystem m_shooter;

    private final CommandPS5Controller m_driverController = new CommandPS5Controller(
            OperatorConstants.kDriverControllerPort);

    public RobotContainer(Bot bot) {
        switch (bot) {
            case COMP:
                configureCompetitionBot();
                break;
            case M1C2:
                configureM1C2();
                break;
            case DOUGHNUT:
                configureDoughnut();
                break;

        }
    }

    private void configureCompetitionBot() {

    }

    private void configureM1C2() {
        m_vision = new VisionSubsystem();
        m_swerveDrive = new SwerveSubsystem(m_vision);
    }

    private void configureDoughnut() {
        m_differentialDrive = new DifferentialDriveSubsystem();
    }

    private void setupShooter() {
        m_shooter = new ShooterSubsystem();

        // Sets the mode of the shooter
        Mode mode = new Mode();
        Command shootCommand = Commands.either(m_shooter.cShootLow(),
                m_shooter.cShootHigh(),
                mode::isAmpMode);

        // Shoot!
        m_driverController.R1().whileTrue(shootCommand);

        // Sets the shooter to amp mode
        m_driverController.square().onTrue(mode.cSetAmpMode());

        // Sets the shooter to speaker mode
        m_driverController.triangle().onTrue(mode.cSetSpeakerMode());

        // sets the shooter to intake from a human player
        m_driverController.L1().whileTrue(m_shooter.cSourceIntake());
    }

    private void setupDifferentialDrive() {
        m_differentialDrive = new DifferentialDriveSubsystem();

        Command teleopDriveCommand = new DifferentialDriveCommand(m_differentialDrive, m_driverController::getLeftY,
                m_driverController::getRightX);

        m_differentialDrive.setDefaultCommand(teleopDriveCommand);
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
