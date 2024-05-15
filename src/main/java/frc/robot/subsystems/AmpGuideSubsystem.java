package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
    private enum AmpGuideState {
        EXTENDED,
        RETRACTED;

        double getSetpoint() {
            switch (this) {
                case EXTENDED:
                    return AmpGuideConstants.kForwardLimit;
                case RETRACTED:
                    return AmpGuideConstants.kRetractedPosition;
                default:
                    return 0;
            }
        }
    }

    private final CANSparkMax ampGuideMotor;
    private AmpGuideState state = AmpGuideState.RETRACTED;

    public AmpGuideSubsystem() {
        ampGuideMotor = new CANSparkMax(Constants.CANConstants.kAmpGuideId, MotorType.kBrushless);

        ampGuideMotor.restoreFactoryDefaults();
        ampGuideMotor.setInverted(true);
        ampGuideMotor.setIdleMode(IdleMode.kBrake);
        ampGuideMotor.setSmartCurrentLimit(20);

        ampGuideMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
        ampGuideMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        ampGuideMotor.setSoftLimit(SoftLimitDirection.kForward, AmpGuideConstants.kForwardLimit);
        ampGuideMotor.enableSoftLimit(SoftLimitDirection.kForward, true);

        ampGuideMotor.getPIDController().setP(2);
        ampGuideMotor.getPIDController().setOutputRange(-0.5, 0.5);

        ampGuideMotor.burnFlash();
    }

    @Override
    public void periodic() {
        ampGuideMotor.getPIDController().setReference(state.getSetpoint(), ControlType.kPosition);
        SmartDashboard.putNumber("Amp Guide/Height", ampGuideMotor.getEncoder().getPosition());
    }

    private Command cSetState(AmpGuideState state) {
        return runOnce(() -> this.state = state);
    }

    public Command cExtend() {
        return cSetState(AmpGuideState.EXTENDED);
    }

    public Command cRetract() {
        return cSetState(AmpGuideState.RETRACTED);
    }
}