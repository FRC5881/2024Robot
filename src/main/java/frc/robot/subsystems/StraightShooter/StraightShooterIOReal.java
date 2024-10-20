package frc.robot.subsystems.StraightShooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.CANConstants;

public class StraightShooterIOReal implements StraightShooterIO {
    private final CANSparkMax shooterTL;
    private final CANSparkMax shooterTR;
    private final CANSparkMax shooterBL;
    private final CANSparkMax shooterBR;
    
    public StraightShooterIOReal() {
        shooterTL = new CANSparkMax(CANConstants.kShooterTL, MotorType.kBrushless);
        shooterTL.setIdleMode(IdleMode.kCoast);
        shooterTR = new CANSparkMax(CANConstants.kShooterTR, MotorType.kBrushless);
        shooterTR.setIdleMode(IdleMode.kCoast);
        shooterBL = new CANSparkMax(CANConstants.kShooterBL, MotorType.kBrushless);
        shooterBL.setIdleMode(IdleMode.kCoast);
        shooterBR = new CANSparkMax(CANConstants.kShooterBR, MotorType.kBrushless);
        shooterBR.setIdleMode(IdleMode.kCoast);
    }

    @Override
    public void setVoltages(double topLeft, double topRight, double bottomLeft, double bottomRight) {
        shooterTL.setVoltage(topLeft);
        shooterTR.setVoltage(topRight);
        shooterBL.setVoltage(bottomLeft);
        shooterBR.setVoltage(bottomRight);
    }

    @Override
    public double getVelocityTL() {
        double veloTL = shooterTL.getEncoder().getVelocity();
        return veloTL;
    }

    @Override
    public double getVelocityTR() {
        double veloTR = shooterTR.getEncoder().getVelocity();
        return veloTR;
    }

    @Override
    public double getVelocityBL() {
        double veloBL = shooterBL.getEncoder().getVelocity();
        return veloBL;
    }

    @Override
    public double getVelocityBR() {
        double veloBR = shooterBR.getEncoder().getVelocity();
        return veloBR;
    }

    @Override
    public double[] getPositions() {
        double posTL = shooterTL.getEncoder().getPosition();
        double posTR = shooterTR.getEncoder().getPosition();
        double posBL = shooterBL.getEncoder().getPosition();
        double posBR = shooterBR.getEncoder().getPosition();
        double[] posArr = {posTL, posTR, posBL, posBR};
        return posArr;
    }
    
}
