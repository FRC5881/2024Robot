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
        NONE(0);
        // TODO: Add more patterns

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
