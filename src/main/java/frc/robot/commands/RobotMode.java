package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * RobotMode keeps track of whether we're in AMP scoring mode of SPEAKER scoring
 * mode.
 */
public class RobotMode {
    private boolean isAmpMode;

    /**
     * Creates a new RobotMode, initialized to AMP mode.
     */
    public RobotMode() {
        isAmpMode = true;
    }

    /**
     * Sets the robot to AMP mode.
     */
    public Command cSetAmpMode() {
        return Commands.runOnce(() -> {
            isAmpMode = true;
        });
    }

    /**
     * Sets the robot to SPEAKER mode.
     */
    public Command cSetSpeakerMode() {
        return Commands.runOnce(() -> {
            isAmpMode = false;
        });
    }

    /**
     * Returns true if the robot is in AMP mode.
     */
    public boolean isAmpMode() {
        return isAmpMode;
    }

    /**
     * Returns true if the robot is in SPEAKER mode.
     */
    public boolean isSpeakerMode() {
        return !isAmpMode;
    }
}
