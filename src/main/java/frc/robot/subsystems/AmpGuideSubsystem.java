package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AmpGuideConstants;

/**
 * AmpGuideSubsystem
 * <p>
 * The AMP Guide uses a State-Machine pattern, it can be in either 1 of 2
 * states. "Extended" or "Retracted". In the periodic method the chosen state is
 * approached using a PID controller.
 */
public class AmpGuideSubsystem extends SubsystemBase {
    private final CANSparkMax ampGuideMotor;

    public enum AmpGuideState {
        RETRACTED,
        EXTENDED,
    }

    private AmpGuideState state = AmpGuideState.RETRACTED;

    /**
     * Get the setpoint for the AMP Guide
     * 
     * @return The setpoint in number of rotations
     */
    private double getSetpoint() {
        switch (state) {
            case EXTENDED:
                return AmpGuideConstants.kForwardLimit;
            case RETRACTED:
                return 0;
        }

        return 0;
    }

    /**
     * Get the current position of the AMP Guide
     * 
     * @return The position in number of rotations
     */
    private double getPosition() {
        return ampGuideMotor.getEncoder().getPosition();
    }

    public AmpGuideSubsystem() {
        ampGuideMotor = new CANSparkMax(Constants.CANConstants.kAmpGuideId, MotorType.kBrushless);
        ampGuideMotor.restoreFactoryDefaults();
        ampGuideMotor.setInverted(true);
        ampGuideMotor.setIdleMode(IdleMode.kBrake);
        ampGuideMotor.setSmartCurrentLimit(20);

        ampGuideMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
        ampGuideMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        ampGuideMotor.setSoftLimit(SoftLimitDirection.kForward, 22.83f);
        ampGuideMotor.enableSoftLimit(SoftLimitDirection.kForward, true);

        ampGuideMotor.getEncoder().setPosition(0.0);
        ampGuideMotor.burnFlash();
    }

    @Override
    public void periodic() {
        ampGuideMotor.getPIDController().setReference(getSetpoint(), ControlType.kPosition);

        // Telemetry
        SmartDashboard.putNumber("Amp Guide/Setpoint", getSetpoint());
        SmartDashboard.putNumber("Amp Guide/Position", getPosition());
        SmartDashboard.putNumber("Amp Guide/Voltage", ampGuideMotor.getBusVoltage() * ampGuideMotor.getAppliedOutput());
    }

    /**
     * Set the state of the AMP Guide
     * 
     * @param state The {@link AmpGuideState} to target
     * @return The {@link Command}
     */
    private Command cSetState(AmpGuideState state) {
        return runOnce(() -> this.state = state);
    }

    /**
     * Set the AMP Guide to the extended state
     * 
     * @return The {@link Command}
     */
    public Command cExtend() {
        return cSetState(AmpGuideState.EXTENDED);
    }

    /**
     * Set the AMP Guide to the retracted state
     * 
     * @return The {@link Command}
     */
    public Command cRetract() {
        return cSetState(AmpGuideState.RETRACTED);
    }

    /**
     * A command that does nothing but wait until the AMP Guide reaches its setpoint
     * 
     * <p>
     * Example usage:
     * 
     * <pre>
     * {@code
     * Command extendAndWait = intake.cExtend().andThen(intake.cWaitForReady());
     * }
     * </pre>
     * 
     * @return The {@link Command}
     */
    public Command cWaitForReady() {
        return Commands.waitUntil(() -> MathUtil.isNear(getSetpoint(), getPosition(), AmpGuideConstants.kTolerance));
    }
}