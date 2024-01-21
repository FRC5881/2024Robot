package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Mode {
    private boolean isAmpMode = true;

    public Mode() {
    }

    public Command cSetAmpMode() {
        return Commands.runOnce(() -> {
            isAmpMode = true;
        });
    }

    public Command cSetSpeakerMode() {
        return Commands.runOnce(() -> {
            isAmpMode = false;
        });
    }

    public boolean isAmpMode() {
        return isAmpMode;
    }

    public boolean isSpeakerMode() {
        return !isAmpMode;
    }
}
