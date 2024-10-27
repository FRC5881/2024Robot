package frc.robot.subsystems.StraightShooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.CANConstants;

public class StraightShooterIOReal implements StraightShooterIO {
    private final CANSparkMax shooterTopLeft;
    private final CANSparkMax shooterTopRight;
    private final CANSparkMax shooterBottomLeft;
    private final CANSparkMax shooterBottomRight;
    
    public StraightShooterIOReal() {
        shooterTopLeft = new CANSparkMax(CANConstants.kShooterTL, MotorType.kBrushless);
        shooterTopLeft.restoreFactoryDefaults();
        shooterTopLeft.setIdleMode(IdleMode.kCoast);
        shooterTopLeft.setInverted(false);

        shooterTopRight = new CANSparkMax(CANConstants.kShooterTR, MotorType.kBrushless);
        shooterTopRight.restoreFactoryDefaults();
        shooterTopRight.setIdleMode(IdleMode.kCoast);
        shooterTopRight.setInverted(true);

        shooterBottomLeft = new CANSparkMax(CANConstants.kShooterBL, MotorType.kBrushless);
        shooterBottomLeft.restoreFactoryDefaults();
        shooterBottomLeft.setIdleMode(IdleMode.kCoast);
        shooterBottomLeft.setInverted(false);

        shooterBottomRight = new CANSparkMax(CANConstants.kShooterBR, MotorType.kBrushless);
        shooterBottomRight.restoreFactoryDefaults();
        shooterBottomRight.setIdleMode(IdleMode.kCoast);
        shooterBottomRight.setInverted(true);
    }

    @Override
    public void setVoltages(double topLeft, double topRight, double bottomLeft, double bottomRight) {
        shooterTopLeft.setVoltage(topLeft);
        shooterTopRight.setVoltage(topRight);
        shooterBottomLeft.setVoltage(bottomLeft);
        shooterBottomRight.setVoltage(bottomRight);
    }

    @Override
    public double getVelocityTL() {
        double veloTL = shooterTopLeft.getEncoder().getVelocity();
        return veloTL;
    }

    @Override
    public double getVelocityTR() {
        double veloTR = shooterTopRight.getEncoder().getVelocity();
        return veloTR;
    }

    @Override
    public double getVelocityBL() {
        double veloBL = shooterBottomLeft.getEncoder().getVelocity();
        return veloBL;
    }

    @Override
    public double getVelocityBR() {
        double veloBR = shooterBottomRight.getEncoder().getVelocity();
        return veloBR;
    }

    @Override
    public double[] getPositions() {
        double posTL = shooterTopLeft.getEncoder().getPosition();
        double posTR = shooterTopRight.getEncoder().getPosition();
        double posBL = shooterBottomLeft.getEncoder().getPosition();
        double posBR = shooterBottomRight.getEncoder().getPosition();
        double[] posArr = {posTL, posTR, posBL, posBR};
        return posArr;
    }
    
}
