package frc.robot.subsystems.StraightShooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class StraightShooterIOSim implements StraightShooterIO {
    // Convert in^2 lbs to KgMetersSquared
    private static final double INCHES_SQUARED_TO_KG_METER_SQUARED = 0.00064516;
    private static final double J_WHEELS = 5 * 0.09016 * INCHES_SQUARED_TO_KG_METER_SQUARED;

    private static final double RPM_TO_RAD_PER_SEC = 2 * Math.PI / 60;

    // top left, top right, bottom left, bottom right
    private final FlywheelSim m_topLeftFlywheelSim = new FlywheelSim(DCMotor.getNEO(1), 1, J_WHEELS);
    private final FlywheelSim m_topRightFlywheelSim = new FlywheelSim(DCMotor.getNEO(1), 1, J_WHEELS);
    private final FlywheelSim m_bottomLeftFlywheelSim = new FlywheelSim(DCMotor.getNEO(1), 1, J_WHEELS);
    private final FlywheelSim m_bottomRightFlywheelSim = new FlywheelSim(DCMotor.getNEO(1), 1, J_WHEELS);

    private static final DCMotor NEO = DCMotor.getNEO(1);

    @Override
    public void setVoltage(double topLeft, double topRight, double bottomLeft, double bottomRight) {
        // Limit input voltage to simulate battery brownout
        double busVoltage = RoboRioSim.getVInVoltage();

        // At most, we can only draw the bus voltage and 40A per motor
        double maxTopLeft = Math.min(busVoltage, NEO.getVoltage(NEO.getTorque(40), getTopLeftVelocity() * RPM_TO_RAD_PER_SEC));
        double maxTopRight = Math.min(busVoltage, NEO.getVoltage(NEO.getTorque(40), getTopRightVelocity() * RPM_TO_RAD_PER_SEC));
        double maxBottomLeft = Math.min(busVoltage, NEO.getVoltage(NEO.getTorque(40), getBottomLeftVelocity() * RPM_TO_RAD_PER_SEC));
        double maxBottomRight = Math.min(busVoltage, NEO.getVoltage(NEO.getTorque(40), getBottomRightVelocity() * RPM_TO_RAD_PER_SEC));

        SmartDashboard.putNumber("/StraightShooter/Top Left Shooter Voltage", NEO.getVoltage(NEO.getTorque(40), getBottomRightVelocity() * RPM_TO_RAD_PER_SEC));
        
        m_topLeftFlywheelSim.setInputVoltage(Math.min(topLeft, maxTopLeft));
        m_topRightFlywheelSim.setInputVoltage(Math.min(topRight, maxTopRight));
        m_bottomLeftFlywheelSim.setInputVoltage(Math.min(bottomLeft, maxBottomLeft));
        m_bottomRightFlywheelSim.setInputVoltage(Math.min(bottomRight, maxBottomRight));
    }

    @Override
    public double getTopLeftVelocity() {
        return m_topLeftFlywheelSim.getAngularVelocityRPM();
    }

    @Override
    public double getTopRightVelocity() {
        return m_topRightFlywheelSim.getAngularVelocityRPM();
    }

    @Override
    public double getBottomLeftVelocity() {
        return m_bottomLeftFlywheelSim.getAngularVelocityRPM();
    }

    @Override
    public double getBottomRightVelocity() {
        return m_bottomRightFlywheelSim.getAngularVelocityRPM();
    }

    public void update(double dt) {
        m_topLeftFlywheelSim.update(dt);
        m_topRightFlywheelSim.update(dt);
        m_bottomLeftFlywheelSim.update(dt);
        m_bottomRightFlywheelSim.update(dt);

        SmartDashboard.putNumber("/StraightShooter/Top Left Shooter Amps", m_topLeftFlywheelSim.getCurrentDrawAmps());

        double simulatedVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(m_topLeftFlywheelSim.getCurrentDrawAmps() + m_topRightFlywheelSim.getCurrentDrawAmps() + m_bottomLeftFlywheelSim.getCurrentDrawAmps() + m_bottomRightFlywheelSim.getCurrentDrawAmps());
        RoboRioSim.setVInVoltage(simulatedVoltage);
    }
}
