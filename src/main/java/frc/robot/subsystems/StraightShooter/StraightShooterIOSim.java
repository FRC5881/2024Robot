    package frc.robot.subsystems.StraightShooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class StraightShooterIOSim implements StraightShooterIO {
    // Convert in^2 lbs to KgMetersSquared
    private static final double INCHES_SQUARED_TO_KG_METER_SQUARED = 0.00064516;
    private static final double J_WHEELS = 5 * 0.09016 * INCHES_SQUARED_TO_KG_METER_SQUARED;

    private static final double RPM_TO_RAD_PER_SEC = 2 * Math.PI / 60;

    // top left, top right, bottom left, bottom right
    private final DCMotorSim m_topLeftFlywheelSim = new DCMotorSim(DCMotor.getNEO(1), 1, J_WHEELS);
    private final DCMotorSim m_topRightFlywheelSim = new DCMotorSim(DCMotor.getNEO(1), 1, J_WHEELS);
    private final DCMotorSim m_bottomLeftFlywheelSim = new DCMotorSim(DCMotor.getNEO(1), 1, J_WHEELS);
    private final DCMotorSim m_bottomRightFlywheelSim = new DCMotorSim(DCMotor.getNEO(1), 1, J_WHEELS);

    private static final DCMotor NEO = DCMotor.getNEO(1);

    @Override
    public void setVoltages(double topLeft, double topRight, double bottomLeft, double bottomRight) {
        // Limit input voltage to simulate battery brownout
        double busVoltage = RoboRioSim.getVInVoltage();

        // At most, we can only draw the bus voltage and 40A per motor. These numbers are always positive
        double tl_40amp_voltage = Math.abs(NEO.getVoltage(NEO.getTorque(Math.signum(topLeft) * 40), getVelocityTL() * RPM_TO_RAD_PER_SEC));
        double tr_40amp_voltage = Math.abs(NEO.getVoltage(NEO.getTorque(Math.signum(topRight) * 40), getVelocityTR() * RPM_TO_RAD_PER_SEC));
        double bl_40amp_voltage = Math.abs(NEO.getVoltage(NEO.getTorque(Math.signum(bottomLeft) * 40), getVelocityBL() * RPM_TO_RAD_PER_SEC));
        double br_40amp_voltage = Math.abs(NEO.getVoltage(NEO.getTorque(Math.signum(bottomRight) * 40), getVelocityBR() * RPM_TO_RAD_PER_SEC));

        // Pick the smallest of bus voltage and max voltages
        double maxTopLeft = Math.min(busVoltage, tl_40amp_voltage);
        double maxTopRight = Math.min(busVoltage, tr_40amp_voltage);
        double maxBottomLeft = Math.min(busVoltage, bl_40amp_voltage);
        double maxBottomRight = Math.min(busVoltage, br_40amp_voltage);

        // Clamp the input voltages to be between -max and +max
        topLeft = MathUtil.clamp(topLeft, -maxTopLeft, maxTopLeft);
        topRight = MathUtil.clamp(topRight, -maxTopRight, maxTopRight);
        bottomLeft = MathUtil.clamp(bottomLeft, -maxBottomLeft, maxBottomLeft);
        bottomRight = MathUtil.clamp(bottomRight, -maxBottomRight, maxBottomRight);
        
        SmartDashboard.putNumber("/StraightShooter/TL Voltage", topLeft);
        SmartDashboard.putNumber("/StraightShooter/TR Voltage", topRight);
        SmartDashboard.putNumber("/StraightShooter/BL Voltage", bottomLeft);
        SmartDashboard.putNumber("/StraightShooter/BR Voltage", bottomRight);

        // Apply the voltage limits after the clamping
        m_topLeftFlywheelSim.setInputVoltage(topLeft);
        m_topRightFlywheelSim.setInputVoltage(topRight);
        m_bottomLeftFlywheelSim.setInputVoltage(bottomLeft);
        m_bottomRightFlywheelSim.setInputVoltage(bottomRight);
    }

    @Override
    public double getVelocityTL() {
        return m_topLeftFlywheelSim.getAngularVelocityRPM();
    }

    @Override
    public double getVelocityTR() {
        return m_topRightFlywheelSim.getAngularVelocityRPM();
    }

    @Override
    public double getVelocityBL() {
        return m_bottomLeftFlywheelSim.getAngularVelocityRPM();
    }

    @Override
    public double getVelocityBR() {
        return m_bottomRightFlywheelSim.getAngularVelocityRPM();
    }

    public void update(double dt) {
        m_topLeftFlywheelSim.update(dt);
        m_topRightFlywheelSim.update(dt);
        m_bottomLeftFlywheelSim.update(dt);
        m_bottomRightFlywheelSim.update(dt);

        // Update the simulated battery voltage
        double simulatedVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(m_topLeftFlywheelSim.getCurrentDrawAmps() + m_topRightFlywheelSim.getCurrentDrawAmps() + m_bottomLeftFlywheelSim.getCurrentDrawAmps() + m_bottomRightFlywheelSim.getCurrentDrawAmps());
        RoboRioSim.setVInVoltage(simulatedVoltage);

        SmartDashboard.putNumber("/StraightShooter/TL Current", m_topLeftFlywheelSim.getCurrentDrawAmps());
        SmartDashboard.putNumber("/StraightShooter/TR Current", m_topRightFlywheelSim.getCurrentDrawAmps());
        SmartDashboard.putNumber("/StraightShooter/BL Current", m_bottomLeftFlywheelSim.getCurrentDrawAmps());
        SmartDashboard.putNumber("/StraightShooter/BR Current", m_bottomRightFlywheelSim.getCurrentDrawAmps());
    }

    @Override
    public double[] getPositions() {
        double tlPosition = m_topLeftFlywheelSim.getAngularPositionRotations();
        double trPosition = m_topRightFlywheelSim.getAngularPositionRotations();
        double blPosition = m_bottomLeftFlywheelSim.getAngularPositionRotations();
        double brPosition = m_bottomRightFlywheelSim.getAngularPositionRotations();

        return new double[] {tlPosition, trPosition, blPosition, brPosition};
    }
}