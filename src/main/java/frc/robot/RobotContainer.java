// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ClimberSubsystem;
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
        m_driverController.povUp().whileTrue(m_climberSubsystem.cExtend());
        m_driverController.povDown().whileTrue(m_climberSubsystem.cRetract());
        m_driverController.cross().whileTrue(m_intakeSubsystem.cRun());
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
