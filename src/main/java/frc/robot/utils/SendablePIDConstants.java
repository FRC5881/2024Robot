package frc.robot.utils;

import com.revrobotics.SparkPIDController;

/**
 * SendablePIDConstants wraps a {@link SparkPIDController} and sends
 * its constants over NetworkTables.
 */
public class SendablePIDConstants {
    private final SparkPIDController m_controller;

    // Cached constants
    private double m_p, m_i, m_d, m_izone, m_ff, m_maxOutput, m_minOutput;

    /**
     * Create a new SendablePIDConstants
     * 
     * @param controller the controller to wrap
     */
    public SendablePIDConstants(SparkPIDController controller) {
        m_controller = controller;

        // Initialize the cached constants with the values from the controller
        m_p = m_controller.getP();
        m_i = m_controller.getI();
        m_d = m_controller.getD();
        m_izone = m_controller.getIZone();
        m_ff = m_controller.getFF();
        m_maxOutput = m_controller.getOutputMax();
        m_minOutput = m_controller.getOutputMin();

        //
    }

}
