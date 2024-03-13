package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.LEDSubsystem.Pattern;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;

/**
 * ShooterSubsystem controls our shooter.
 * <p>
 * The shooter has 2 independent flywheels, a main and a secondary. The main
 * flywheel is at the top of the robot, secondary is slightly lower.
 * <p>
 * The main and secondary have similar enough performance that we can use
 * identical PID and feedforward constants for both.
 * <p>
 * The {@link IntakeSubsystem} passes NOTES into the shooter.
 * <p>
 * The shooter can be used in 2 modes:
 * <ul>
 * <li>Full power mode: where 100% power is applied to the motors, creating the
 * maximum (but inconsistent) launch speed.
 * <li>Controlled mode: where the motors are controlled by a PID loop to
 * maintain a consistent speed throughout the match.
 * </ul>
 * <p>
 * The shooter can also be used to intake NOTES from the source, in which case
 * the motors are spun slowly in reverse.
 */
public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax mainMotor;
    private final CANSparkMax secondaryMotor;

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0.12608, 0.0080106);
    private final PIDController pidController = new PIDController(0.05, 0, 0);

    public ShooterSubsystem() {
        mainMotor = new CANSparkMax(Constants.CANConstants.kShooterMainId, MotorType.kBrushless);
        mainMotor.restoreFactoryDefaults();
        mainMotor.setSmartCurrentLimit(40);
        mainMotor.setIdleMode(IdleMode.kBrake);
        // Convert from RPM to RPS
        mainMotor.getEncoder().setVelocityConversionFactor(1.0 / 60.0);

        secondaryMotor = new CANSparkMax(Constants.CANConstants.kShooterSecondaryId, MotorType.kBrushless);
        secondaryMotor.restoreFactoryDefaults();
        secondaryMotor.setSmartCurrentLimit(40);
        secondaryMotor.setIdleMode(IdleMode.kBrake);
        // Convert from RPM to RPS
        secondaryMotor.getEncoder().setVelocityConversionFactor(1.0 / 60.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/MainRPS", mainMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter/SecondaryRPS", secondaryMotor.getEncoder().getVelocity());

        SmartDashboard.putNumber("Shooter/MainVoltage", mainMotor.getAppliedOutput() * mainMotor.getBusVoltage());
        SmartDashboard.putNumber("Shooter/SecondaryVoltage",
                secondaryMotor.getAppliedOutput() * secondaryMotor.getBusVoltage());

        SmartDashboard.putNumber("Shooter/MainCurrent", mainMotor.getOutputCurrent());
        SmartDashboard.putNumber("Shooter/SecondaryCurrent", secondaryMotor.getOutputCurrent());
    }

    // ---- CONTROLS ---- //

    /**
     * Stops the shooter subsystem flywheels.
     */
    private void stop() {
        mainMotor.stopMotor();
        secondaryMotor.stopMotor();
    }

    /**
     * Drives the shooter motors with a given power.
     * 
     * @param percentage the values to drive the motors at, between -1 and 1.
     * @return the command to run
     */
    private Command cPercentOutput(Measure<Dimensionless> percentage) {
        return runEnd(() -> {
            mainMotor.set(percentage.in(Value));
            secondaryMotor.set(percentage.in(Value));
        }, this::stop);
    }

    /**
     * Has the shooter target the SPEAKER.
     * 
     * @return the command to run
     */
    public Command cRunSpeaker() {
        return cPercentOutput(Percent.of(100)).raceWith(LEDSubsystem.cSetOverride(Pattern.CHASING_UP));
    }

    public Command cStop() {
        return runOnce(this::stop);
    }

    /**
     * Drives the shooter motors at a given setpoint.
     * <p>
     * Uses a feedforward and PID controller to set the voltage of the motors.
     */
    private Command cSetpoint(Measure<Velocity<Angle>> setpoint) {
        return runEnd(() -> {
            double target = setpoint.in(RotationsPerSecond);
            double mainSpeed = mainMotor.getEncoder().getVelocity();
            double secondarySpeed = secondaryMotor.getEncoder().getVelocity();

            mainMotor.setVoltage(feedforward.calculate(target) + pidController.calculate(mainSpeed, target));
            secondaryMotor.setVoltage(feedforward.calculate(target) + pidController.calculate(secondarySpeed, target));
        }, this::stop);
    }

    /**
     * Has the shooter target the AMP.
     * 
     * @return the command to run
     */
    public Command cRunAmp() {
        return cSetpoint(ShooterConstants.kShooterAmpSpeed).raceWith(LEDSubsystem.cSetOverride(Pattern.CHASING_UP));
    }

    /**
     * Checks if the shooter motors are at the specified setpoint velocity.
     * 
     * @param setpoint The desired setpoint velocity.
     * @return True if both main and secondary motors are near the setpoint velocity
     *         within the tolerance, false otherwise.
     */
    private boolean isAtSetpoint(Measure<Velocity<Angle>> setpoint) {
        double mainSpeed = mainMotor.getEncoder().getVelocity();
        double secondarySpeed = secondaryMotor.getEncoder().getVelocity();
        double rps = setpoint.in(RotationsPerSecond);
        double tolerance = ShooterConstants.kShooterTolerance.in(RotationsPerSecond);

        return MathUtil.isNear(mainSpeed, rps, tolerance) && MathUtil.isNear(secondarySpeed, rps, tolerance);
    }

    /**
     * A command that does nothing, but finishes when the shooter is at the
     * specified setpoint velocity, or after a timeout.
     * 
     * @param setpoint The desired setpoint velocity.
     * @return The command to run.
     */
    private Command waitForReady(Measure<Velocity<Angle>> setpoint) {
        // TODO: https://github.com/wpilibsuite/allwpilib/pull/6386
        return Commands.waitUntil(() -> isAtSetpoint(setpoint)).withTimeout(ShooterConstants.kTimeout.in(Seconds));
    }

    /**
     * A command that does nothing, but finishes when the shooter is at the AMP
     * setpoint velocity, or after a timeout.
     * 
     * @return The command to run.
     */
    public Command waitForAmpReady() {
        return waitForReady(ShooterConstants.kShooterAmpSpeed);
    }

    /**
     * Runs the given {@link Command} when the shooter's ready to score in the AMP.
     * 
     * @param command The command to run when the shooter is ready.
     * @return The composed command.
     */
    public Command cRunWhenAmpReady(Command command) {
        return cRunAmp().alongWith(waitForAmpReady().andThen(command));
    }

    /**
     * Runs the given {@link Command} when the shooter's ready to score in the
     * SPEAKER.
     * 
     * @param command The command to run when the shooter is ready.
     * @return The composed command.
     */
    public Command cRunWhenSpeakerReady(Command command) {
        // TODO: https://github.com/wpilibsuite/allwpilib/pull/6386
        return cRunSpeaker().alongWith(Commands.waitSeconds(ShooterConstants.kTimeout.in(Seconds)).andThen(command));
    }

    /**
     * Runs the shooter in reverse to intake a NOTE.
     * 
     * @return The command to run.
     */
    public Command cIntake() {
        return cPercentOutput(Percent.of(-15)).raceWith(LEDSubsystem.cSetOverride(Pattern.CHASING_DOWN));
    }

    // ---- SYSTEM IDENTIFICATION ---- //

    private final MutableMeasure<Voltage> mainVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Angle> mainPosition = mutable(Rotation.of(0));
    private final MutableMeasure<Velocity<Angle>> mainVelocity = mutable(RotationsPerSecond.of(0));

    private final MutableMeasure<Voltage> secondaryVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Angle> secondaryPosition = mutable(Rotation.of(0));
    private final MutableMeasure<Velocity<Angle>> secondaryVelocity = mutable(RotationsPerSecond.of(0));

    private void voltageDrive(Measure<Voltage> voltage) {
        mainMotor.setVoltage(voltage.in(Volts));
        secondaryMotor.setVoltage(voltage.in(Volts));
    }

    private void logMotors(SysIdRoutineLog log) {
        mainVoltage.mut_replace(mainMotor.getAppliedOutput() * mainMotor.getBusVoltage(), Volts);
        mainPosition.mut_replace(mainMotor.getEncoder().getPosition(), Rotations);
        mainVelocity.mut_replace(mainMotor.getEncoder().getVelocity(), Rotations.per(Minute));

        secondaryVoltage.mut_replace(secondaryMotor.getAppliedOutput() * secondaryMotor.getBusVoltage(), Volts);
        secondaryPosition.mut_replace(secondaryMotor.getEncoder().getPosition(), Rotations);
        secondaryVelocity.mut_replace(secondaryMotor.getEncoder().getVelocity(), Rotations.per(Minute));

        log.motor("main")
                .voltage(mainVoltage)
                .angularPosition(mainPosition)
                .angularVelocity(mainVelocity);

        log.motor("secondary")
                .voltage(secondaryVoltage)
                .angularPosition(secondaryPosition)
                .angularVelocity(secondaryVelocity);
    }

    private final SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(this::voltageDrive, this::logMotors, this));

    public Command sysId() {
        return Commands.sequence(
                routine.quasistatic(Direction.kForward),
                Commands.waitSeconds(1),
                routine.quasistatic(Direction.kReverse),
                Commands.waitSeconds(1),
                routine.dynamic(Direction.kForward),
                Commands.waitSeconds(1),
                routine.dynamic(Direction.kReverse));
    }
}
