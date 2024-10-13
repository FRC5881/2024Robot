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
    public void setVoltage(double topLeft, double topRight, double bottomLeft, double bottomRight);
    
    /**
     * Sets the desired voltages of all 4 shooter motors using an array.
     * 
     * @param voltages [topLeft, topRight, bottonLeft, bottonRight].
     * @throws IllegalArgumentException
     */
    default public void setVoltage(double[] voltages) throws IllegalArgumentException {
        if (voltages.length != 4) {
            throw new IllegalArgumentException("StraightShooter has 4 motors and therefor requires 4 voltages");
        }

        this.setVoltage(voltages[0], voltages[1], voltages[2], voltages[3]);
    }

    default void stop() {
        setVoltage(0, 0, 0, 0);
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
    public double getTopLeftVelocity();

    /**
     * Returns the rotational velocity of the top right shooter.
     * @return rotational velocity, RPMs
     */
    public double getTopRightVelocity();

    /**
     * Returns the rotational velocity of the bottom left shooter.
     * @return rotational velocity, RPMs
     */
    public double getBottomLeftVelocity();

    /**
     * Returns the rotational velocity of the bottom right shooter.
     * @return rotational velocity, RPMs
     */
    public double getBottomRightVelocity();

    /**
     * Returns the current rotational velocity of all 4 motors. In rotations per minute.
     * 
     * @return [topLeftRPM, topRightRPM, bottomLeftRPM, bottomRightRPM]
     */
    default public double[] getVelocities() {
        return new double[] { getTopLeftVelocity(), getTopRightVelocity(), getBottomLeftVelocity(), getBottomRightVelocity() };
    }
}
