package frc.robot.utils;

import edu.wpi.first.wpilibj.AnalogOutput;

/**
 * PenningtonLEDs is the low-level interface to control our custom-made LED
 * controller.
 */
public class PenningtonLEDs {
    private AnalogOutput m_analogOutput;

    public PenningtonLEDs(int channel) {
        m_analogOutput = new AnalogOutput(channel);
    }

    public enum RawPattern {
        SLOW_RAINBOW(0),
        SOLID_RED(1),
        SOLID_GREEN(2),
        SOLID_BLUE(3),
        BREATHING_RED(4),
        BREATHING_GREEN(5),
        BREATHING_BLUE(6),
        SLOW_FLASH_GREEN(7),
        CHASING_UP_RED(8),
        CHASING_UP_GREEN(9),
        CHASING_UP_BLUE(10),
        CHASING_DOWN_RED(11),
        CHASING_DOWN_GREEN(12),
        CHASING_DOWN_BLUE(13),
        FAST_FLASH_RED(14),
        FAST_FLASH_GREEN(15),
        FAST_FLASH_BLUE(16),
        FAST_RAINBOW_FLASH(17);

        private final int id;

        private RawPattern(int id) {
            this.id = id;
        }

        public int getId() {
            return id;
        }
    }

    /**
     * Calculates the voltage to send to the LED controller to display a pattern
     * 
     * @param pattern the {@link RawPattern} to display
     * @return the voltage to send to the LED controller
     */
    private static double getVoltage(RawPattern pattern) {
        return 5.0 * (pattern.getId() + 0.5) / RawPattern.values().length;
    }

    /**
     * Sets the desired pattern for the LEDs to display.
     * <p>
     * This pattern is held until a new pattern is set.
     * <p>
     * Note: The new pattern only takes effect after the previous pattern's cycle
     * has completed
     * 
     * @param pattern the {@link RawPattern} to display
     */
    public void setPattern(RawPattern pattern) {
        m_analogOutput.setVoltage(getVoltage(pattern));
    }
}
