package frc.robot.subsystems.StraightShooter;

public interface StraightShooterIO {

    // VOLTAGE STUFF
    public void setVoltages(double topLeft, double topRight, double bottomLeft, double bottomRight);

    default public void setVoltages(double[] voltages) throws IllegalArgumentException {
        if (voltages.length != 4) {
            throw new IllegalArgumentException("StraightShooter has 4 motors and therefor requires 4 voltages");
        }
        this.setVoltages(voltages[0], voltages[1], voltages[2], voltages[3]);
    }

    default void stop() {
        setVoltages(0, 0, 0, 0);
    }

    public double getVoltageTL();
    public double getVoltageTR();
    public double getVoltageBL();
    public double getVoltageBR();

    default public double[] getVoltages() {
        return new double[] { getVoltageTL(), getVoltageTR(), getVoltageBL(), getVoltageBR() };
    }


    //FEEDFORWARD STUFF
    public double getFFks();
    public double getFFkv();
    public double getFFka();

    
    //PID STUFF
    public double getPIDkp();
    public double getPIDki();
    public double getPIDkd();
 

    //VELOCITY STUFF
    public double getVelocityTL();
    public double getVelocityTR();
    public double getVelocityBL();
    public double getVelocityBR();

    default public double[] getVelocities() {
        return new double[] { getVelocityTL(), getVelocityTR(), getVelocityBL(), getVelocityBR() };
    }
}
