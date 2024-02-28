package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.controller.*;

public class ShooterClimberSubsystem extends SubsystemBase {
    private final CANSparkMax shooterMotor1;
    private final CANSparkMax shooterMotor2;
    private final Encoder shooterAngleEncoder;
    private final PIDController shooterAngleController;
    private final CANSparkMax shooterAngleMotor1;
    private final CANSparkMax shooterAngleMotor2;

    public ShooterClimberSubsystem() {
        shooterMotor1 = new CANSparkMax(Constants.ShooterConstants.kShooterMotor1Port, MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(Constants.ShooterConstants.kShooterMotor2Port, MotorType.kBrushless);
        shooterAngleMotor1 = new CANSparkMax(Constants.ShooterConstants.kShooterAngleMotor1Port, MotorType.kBrushless);
        shooterAngleMotor2 = new CANSparkMax(Constants.ShooterConstants.kShooterAngleMotor2Port, MotorType.kBrushless);
        shooterMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        shooterMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);
        shooterAngleMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        shooterAngleMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);
        shooterAngleEncoder = new Encoder(0, 1);
        shooterAngleController = new PIDController(0.1, 0.1, 0.1);

    }

    public void SetAngle(double targetAngle) {
        /* Calculates speeds to reach target angle */
        double currentAngle = shooterAngleEncoder.get();
        double output = shooterAngleController.calculate(currentAngle, targetAngle);

        /* Sets the motor speeds to reach target angle */
        shooterAngleMotor1.set(output);
        shooterAngleMotor2.set(-output);

    }

    public void shootOnSpeaker() {
        shooterMotor1.set(1);
        shooterMotor2.set(-1);
    }

    public void shootOnAmp() {
        shooterMotor1.set(.25);
        shooterMotor2.set(-.25);
    }

    public void stopShooter() {
        shooterMotor1.set(0);
        shooterMotor2.set(0);
    }

    public Command cShootOnSpeaker() {
        return this.runEnd(this::shootOnSpeaker, this::stopShooter);
    }

    public Command cShootOnAmp() {
        return this.runEnd(this::shootOnAmp, this::stopShooter);
    }

}
