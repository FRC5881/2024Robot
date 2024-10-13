package frc.robot.subsystems.StraightShooter;

public interface StraightShooterIO {
    /**
     * Sets the desired voltage for all 4 shooter motors. Should be called once per tick.
     * 
     * @param topLeft
     * @param topRight
     * @param bottomLeft
     * @param bottomRight
     */
    public void setVoltages(double topLeft, double topRight, double bottomLeft, double bottomRight);
    
    /**
     * Sets the desired voltages of all 4 shooter motors using an array.
     * 
     * @param voltages [topLeft, topRight, bottonLeft, bottonRight].
     * @throws IllegalArgumentException
     */
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


    public double getFFks();
    public double getFFkv();
    public double getFFka();

    public double getPIDkp();
    public double getPIDki();
    public double getPIDkd();


    /**
     * Returns the rotational velocity of the top left shooter.
     * @return rotational velocity, RPMs
     */
    public double getVelocityTL();

    /**
     * Returns the rotational velocity of the top right shooter.
     * @return rotational velocity, RPMs
     */
    public double getVelocityTR();

    /**
     * Returns the rotational velocity of the bottom left shooter.
     * @return rotational velocity, RPMs
     */
    public double getVelocityBL();

    /**
     * Returns the rotational velocity of the bottom right shooter.
     * @return rotational velocity, RPMs
     */
    public double getVelocityBR();

    /**
     * Returns the current rotational velocity of all 4 motors. In rotations per minute.
     * 
     * @return [topLeftRPM, topRightRPM, bottomLeftRPM, bottomRightRPM]
     */
    default public double[] getVelocities() {
        return new double[] { getVelocityTL(), getVelocityTR(), getVelocityBL(), getVelocityBR() };
    }
}
