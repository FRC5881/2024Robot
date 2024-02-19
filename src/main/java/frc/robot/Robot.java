// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    // DigitalOutput pwm = new DigitalOutput(0);
    // Spark spark = new Spark(0);
    PWM pwm = new PWM(0);
    DigitalOutput dio = new DigitalOutput(0);
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    private AnalogInput A0 = new AnalogInput(0);
    private AnalogInput A1 = new AnalogInput(1);

    /**
     * Our code is designed to work with multiple robots. This enum is used to
     * represent which robot we are currently running on.
     */
    public enum RobotFrame {
        /**
         * Our Competion bot
         * 
         * LOW - LOW
         */
        COMP,
        /**
         * 2023 Robot as a test bed this year
         * 
         * HIGH - LOW
         */
        M1C2,
        /**
         * Andymark KOP chassis
         * 
         * LOW - HIGH
         */
        DOUGHNUT,
    }

    public RobotFrame detectChassis() {
        boolean a0 = A0.getVoltage() >= 2.5;
        boolean a1 = A1.getVoltage() >= 2.5;

        if (!a0 && !a1) {
            return RobotFrame.COMP;
        } else if (a0 && !a1) {
            return RobotFrame.M1C2;
        } else if (!a0 && a1) {
            return RobotFrame.DOUGHNUT;
        } else {
            throw new RuntimeException("Unknown robot configuration");
        }
    }

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
        // m_robotContainer = new RobotContainer(detectChassis());
        m_robotContainer = new RobotContainer(RobotFrame.COMP);
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics that you want ran during disabled, autonomous, teleoperated
     * and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        // Put analog input values on the dashboard
        SmartDashboard.putNumber("A0", A0.getValue());
        SmartDashboard.putNumber("A1", A1.getValue());
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }
}
