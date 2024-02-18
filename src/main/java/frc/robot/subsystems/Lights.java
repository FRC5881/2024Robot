package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.AnalogOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

// Voltages are all private information
public class Lights {
    private static Lights instance = null;

    private Lights() {
    }

    public static Lights getInstance() {
        if (Lights.instance == null) {
            Lights.instance = new Lights();
        }
        return instance;
    }

    AnalogOutput analogOutput = new AnalogOutput(0);

    public enum Pattern {
        RED_STROBE(0),
        RED_SOLID(1),
        RED_FLASH(2),
        RED_CHASE(3),
        RED_REPEATED_FLASH(4),
        GREEN_STROBE(5),
        GREEN_SOLID(6),
        GREEN_FLASH(7),
        GREEN_CHASE(8),
        GREEN_REPEATED_FLASH(9),
        BLUE_STROBE(10),
        BLUE_SOLID(11),
        BLUE_FLASH(12),
        BLUE_CHASE(13),
        BLUE_REPEATED_FLASH(14);

        private final int value;

        Pattern(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }

        @Override
        public String toString() {
            return this.name().replace('_', ' ').toLowerCase();
        }
    }

    private static double patternToVoltage(Pattern pattern) {
        return 5.0 / 15.0 * (pattern.value + 0.5);
    }

    private void setPattern() {
        if (overridePattern == null) {
            analogOutput.setVoltage(patternToVoltage(defaultPattern));
        } else {
            analogOutput.setVoltage(patternToVoltage(overridePattern));
        }

    }

    private Pattern defaultPattern = Pattern.GREEN_SOLID;

    public void setDefault() {
        Optional<Alliance> ally = DriverStation.getAlliance();

        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                this.defaultPattern = Pattern.RED_SOLID;
            } else {
                this.defaultPattern = Pattern.BLUE_SOLID;
            }
        } else {
            this.defaultPattern = Pattern.GREEN_SOLID;
        }
        setPattern();
    }

    private Pattern overridePattern = null;

    public void startOverride(Pattern pattern) {
        overridePattern = pattern;
        setPattern();
    }

    public void endOverride(Pattern pattern) {
        if (pattern == overridePattern) {
            overridePattern = null;
        }
        setPattern();
    }

}
