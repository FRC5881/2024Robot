package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
    // 2 analog sensors to detect the NOTE

    private boolean hasNoteBottom() {
        return false;
    }

    private boolean hasNoteTop() {
        return false;
    }

    /**
     * Moves a note higher through the mechanism
     */
    private void up() {
        // control a motor
    }

    private void down() {
        // -up
    }

    /**
     * Stops the indexer
     */
    private void stop() {

    }

    /**
     * When a note enters the Indexer, it is moved up to the top sensor
     */
    public Command cPositionNote() {
        return this.run(() -> {
            if (hasNoteBottom() && !hasNoteTop()) {
                up();
            } else {
                stop();
            }
        });
    }

    // Override the sensor, and send the NOTE to the shooter!
    public Command cSendShooter() {
        return this.runEnd(this::up, this::stop);
    }

    public Command cSendIntake() {
        return this.runEnd(this::down, this::stop);
    }
}
