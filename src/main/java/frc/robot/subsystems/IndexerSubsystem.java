package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {
    private final CANSparkMax indexerMotor;

    public IndexerSubsystem() {
        indexerMotor = new CANSparkMax(Constants.CANConstants.kIndexerMotor, MotorType.kBrushless);
        indexerMotor.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Moves a note higher through the mechanism
     */
    private void up() {
        indexerMotor.set(Constants.IndexerConstants.kIndexerPower);
    }

    private void down() {
        indexerMotor.set(-Constants.IndexerConstants.kIndexerPower);
    }

    /**
     * Stops the indexer
     */
    private void stop() {
        indexerMotor.set(0);
    }

    // Override the sensor, and send the NOTE to the shooter!
    public Command cSendShooter() {
        return this.runEnd(this::up, this::stop);
    }

    public Command cSendDown() {
        return this.runEnd(this::down, this::stop);
    }

}
