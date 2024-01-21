// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Mode;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class RobotContainer {
    private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
    private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(m_visionSubsystem);
    private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

    public RobotContainer() {
        configureBindings();
    }

    private final CommandPS5Controller m_driverController = new CommandPS5Controller(
            OperatorConstants.kDriverControllerPort);

    private void configureBindings() {
        m_driverController.povUp().whileTrue(m_climber.cExtend());
        m_driverController.povDown().whileTrue(m_climber.cRetract());

        // Shooter

        // Sets the mode of the shooter
        Mode mode = new Mode();
        Command shootCommand = Commands.either(m_shooter.cShootLow(), m_shooter.cShootHigh(), mode::isAmpMode);
        m_driverController.R1().whileTrue(shootCommand);
        // Sets the shooter to amp mode
        m_driverController.square().onTrue(mode.cSetAmpMode());
        // Sets the shooter to speaker mode
        m_driverController.triangle().onTrue(mode.cSetSpeakerMode());

        // sets the shooter to intake from a human player
        m_driverController.L1().whileTrue(m_shooter.cSourceIntake());

        // extends the climber
        m_driverController.povUp().whileTrue(m_climberSubsystem.cExtend());
        // retracts the climber
        m_driverController.povDown().whileTrue(m_climberSubsystem.cRetract());
        // activates the ground intake
        m_driverController.cross().whileTrue(m_intakeSubsystem.cRun());
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
