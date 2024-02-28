package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.PenningtonLEDs;
import frc.robot.utils.PenningtonLEDs.RawPattern;

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
        SLOW_RAINBOW,
        SOLID,
        BREATHING,
        POWER_DOWN,
        SLOW_FLASH_GREEN,
        CHASING_UP,
        CHASING_DOWN,
        FAST_FLASH,
        SOLID_PURPLE,
        FAST_RAINBOW_FLASH;

        @Override
        public String toString() {
            return this.name().replace('_', ' ').toLowerCase();
        }
    }

    public enum AllianceColor {
        RED,
        BLUE,
        GREEN,
    }

    private static AllianceColor getColor() {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
                return AllianceColor.RED;
            } else {
                return AllianceColor.BLUE;
            }
        } else {
            return AllianceColor.GREEN;
        }
    }

    /**
     * Converts a high level {@link Pattern} to a low level {@link RawPattern}
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
    private static RawPattern toRaw(Pattern pattern, AllianceColor alliance) {
        switch (pattern) {
            case BREATHING:
                switch (alliance) {
                    case BLUE:
                        return RawPattern.BREATHING_BLUE;
                    case GREEN:
                        return RawPattern.BREATHING_GREEN;
                    case RED:
                        return RawPattern.BREATHING_RED;
                }
            case CHASING_DOWN:
                switch (alliance) {
                    case BLUE:
                        return RawPattern.CHASING_DOWN_BLUE;
                    case GREEN:
                        return RawPattern.CHASING_DOWN_GREEN;
                    case RED:
                        return RawPattern.CHASING_DOWN_RED;
                }
            case CHASING_UP:
                switch (alliance) {
                    case BLUE:
                        return RawPattern.CHASING_UP_BLUE;
                    case GREEN:
                        return RawPattern.CHASING_UP_GREEN;
                    case RED:
                        return RawPattern.CHASING_UP_RED;
                }
            case FAST_FLASH:
                switch (alliance) {
                    case BLUE:
                        return RawPattern.FAST_FLASH_BLUE;
                    case GREEN:
                        return RawPattern.FAST_FLASH_GREEN;
                    case RED:
                        return RawPattern.FAST_FLASH_RED;
                }
            case FAST_RAINBOW_FLASH:
                return RawPattern.FAST_RAINBOW_FLASH;
            case POWER_DOWN:
                return RawPattern.POWER_DOWN;
            case SLOW_FLASH_GREEN:
                return RawPattern.SLOW_FLASH_GREEN;
            case SLOW_RAINBOW:
                return RawPattern.SLOW_RAINBOW;
            case SOLID:
                switch (alliance) {
                    case BLUE:
                        return RawPattern.SOLID_BLUE;
                    case GREEN:
                        return RawPattern.SOLID_GREEN;
                    case RED:
                        return RawPattern.SOLID_RED;
                }
            case SOLID_PURPLE:
                return RawPattern.SOLID_PURPLE;
            default:
                return RawPattern.SLOW_RAINBOW;
        }
    }

    private Pattern defaultPattern = Pattern.SLOW_RAINBOW;
    private Optional<Pattern> overridePattern = Optional.empty();

    public void setDefault(Pattern pattern) {
        defaultPattern = pattern;
        sendPattern();
    }

    public void startOverride(Pattern pattern) {
        overridePattern = Optional.of(pattern);
        sendPattern();
    }

    public void endOverride(Pattern pattern) {
        if (overridePattern.equals(Optional.of(pattern))) {
            overridePattern.filter(pattern::equals).isPresent();
        }
        sendPattern();
    }

    private void sendPattern() {
        Pattern p = overridePattern.orElse(defaultPattern);
        RawPattern r = toRaw(p, getColor());
        leds.setPattern(r);
    }

    public Command cPattern(Pattern pattern) {
        return Commands.runEnd(() -> startOverride(pattern), () -> endOverride(pattern));
    }

    public Command cCrazy() {
        return cPattern(Pattern.FAST_RAINBOW_FLASH).withTimeout(2);
    }
}
