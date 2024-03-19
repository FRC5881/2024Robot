package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.utils.PenningtonLEDs;
import frc.robot.utils.PenningtonLEDs.RawPattern;

public class LEDSubsystem implements Subsystem {
    private static final PenningtonLEDs ledController = new PenningtonLEDs(0);
    private static final LEDSubsystem instance = new LEDSubsystem();

    private LEDSubsystem() {
    }

    public enum Pattern {
        SLOW_RAINBOW,
        SOLID,
        BREATHING,
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
     * {@code Pattern.SOLID} may be converted to {@code RawPattern.SOLID_RED} or
     * {@code RawPattern.SOLID_BLUE} depending on the alliance color
     * 
     * @param pattern  the high level {@link Pattern} to display
     * @param alliance our alliance color (if known)
     * 
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

    private static Pattern defaultPattern = Pattern.SLOW_RAINBOW;
    private static Optional<Pattern> overridePattern = Optional.empty();

    /**
     * Sends the currently active pattern to the LED controller
     */
    private static void sendPattern() {
        Pattern p = overridePattern.orElse(defaultPattern);
        RawPattern r = toRaw(p, getColor());
        ledController.setPattern(r);
    }

    /**
     * Sets the default pattern to display
     * 
     * @param pattern the {@link Pattern} to display
     */
    public static void setDefault(Pattern pattern) {
        defaultPattern = pattern;
        sendPattern();
    }

    /**
     * Sets the default pattern to display
     * 
     * @param pattern
     */
    public static Command cSetDefault(Pattern pattern) {
        return instance.runOnce(() -> setDefault(pattern));
    }

    /**
     * Starts an override pattern
     * 
     * @param pattern the {@link Pattern} to display
     */
    public static void startOverride(Pattern pattern) {
        overridePattern = Optional.of(pattern);
        sendPattern();
    }

    /**
     * Ends the current override pattern
     * 
     * @param pattern the {@link Pattern} to display
     */
    public static void endOverride(Pattern pattern) {
        if (overridePattern.equals(Optional.of(pattern))) {
            overridePattern.filter(pattern::equals).isPresent();
        }
        sendPattern();
    }

    /**
     * Creates a command that sets the override pattern
     * <p>
     * This command runs forever, so it's important to pair it with a command that
     * ends the override
     * 
     * @param pattern the {@link Pattern} to display
     * @return The {@link Command}
     */
    public static Command cSetOverride(Pattern pattern) {
        return instance.runEnd(() -> startOverride(pattern), () -> endOverride(pattern)).withName(pattern.toString())
                .ignoringDisable(true);
    }

    /**
     * Creates a command that sets the override pattern for a limited time
     * <p>
     * This command runs for a limited time, and will automatically end the override
     * 
     * @param pattern the {@link Pattern} to display
     * @param seconds the number of seconds to display the pattern for
     * @return The {@link Command}
     * 
     */
    public static Command cSetOverride(Pattern pattern, double seconds) {
        return cSetOverride(pattern).withTimeout(seconds);
    }

    /**
     * Creates a command that sets the override pattern until a condition becomes
     * true
     * <p>
     * This command runs until the condition is false, and will automatically end
     * the override
     * 
     * @param pattern   the {@link Pattern} to display
     * @param condition the condition to check
     * @return The {@link Command}
     */
    public static Command cSetOverride(Pattern pattern, BooleanSupplier condition) {
        return cSetOverride(pattern).until(condition);
    }
}
