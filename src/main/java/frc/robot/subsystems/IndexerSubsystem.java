package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Value;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.CANConstants;

public class IndexerSubsystem extends SubsystemBase {
    private final CANSparkMax indexerMotor;

    public IndexerSubsystem() {
        indexerMotor = new CANSparkMax(CANConstants.kIndexerMotor, MotorType.kBrushless);
        indexerMotor.restoreFactoryDefaults();
        indexerMotor.setIdleMode(IdleMode.kBrake);
        indexerMotor.setInverted(true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Indexer/Voltage", indexerMotor.getAppliedOutput() * indexerMotor.getBusVoltage());
        SmartDashboard.putNumber("Indexer/Current", indexerMotor.getOutputCurrent());
        SmartDashboard.putNumber("Indexer/Velocity", indexerMotor.getEncoder().getVelocity());
    }

    private void up() {
        indexerMotor.set(IndexerConstants.kIndexerPower.in(Value));
    }

    private void down() {
        indexerMotor.set(-IndexerConstants.kIndexerPower.in(Value));
    }

    public Command cSendShooter() {
        return this.runEnd(this::up, indexerMotor::stopMotor);
    }

    public Command cSendDown() {
        return this.runEnd(this::down, indexerMotor::stopMotor);
    }
}
