package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.utils.PenningtonLEDs;
import frc.robot.utils.PenningtonLEDs.RawPattern;

// Voltages are all private information
public class LEDSubsystem {
    private static LEDSubsystem instance = null;
    private final PenningtonLEDs leds = new PenningtonLEDs(0);

    private LEDSubsystem() {
    }

    /**
     * Gets the singleton instance of the LEDSubsystem
     * 
     * @return the LEDSubsystem
     */
    public static LEDSubsystem getInstance() {
        if (LEDSubsystem.instance == null) {
            LEDSubsystem.instance = new LEDSubsystem();
        }
        return instance;
    }

    public enum Pattern {
        NONE(0);
        // TODO: Add more patterns

        private final int value;

        private Pattern(int value) {
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

    /**
     * Converts a high level Pattern to a low level RawPattern
     * <p>
     * This method typically just adds color information to an LED movement
     * definition.
     * <p>
     * Pattern.SOLID may be converted to RawPattern.SOLID_RED or
     * RawPattern.SOLID_BLUE depending on the alliance color
     * 
     * @param pattern  the high level {@link Pattern} to display
     * @param alliance our alliance color (if known)
     * @return the low level {@link RawPattern} to display
     */
    private static RawPattern toRaw(Pattern pattern, Optional<Alliance> alliance) {
        return RawPattern.NONE;
    }

    private Pattern defaultPattern = Pattern.NONE;
    private Optional<Pattern> overridePattern = Optional.empty();

    public void setDefault(Pattern pattern) {
    }

    public void startOverride(Pattern pattern) {
    }

    public void endOverride(Pattern pattern) {
    }
}
