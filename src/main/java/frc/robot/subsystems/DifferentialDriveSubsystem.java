package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.CANConstants;

public class DifferentialDriveSubsystem extends SubsystemBase {
    private final DifferentialDriveKinematics kinematics;

    private final CANSparkMax leftMain = new CANSparkMax(CANConstants.kLeftMainId, MotorType.kBrushed);
    private final CANSparkMax leftSecond = new CANSparkMax(CANConstants.kLeftSecondId, MotorType.kBrushed);

    private final CANSparkMax rightMain = new CANSparkMax(CANConstants.kRightMainId, MotorType.kBrushed);
    private final CANSparkMax rightSecond = new CANSparkMax(CANConstants.kRightSecondId, MotorType.kBrushed);

    public DifferentialDriveSubsystem() {
        leftMain.restoreFactoryDefaults();
        leftMain.setSmartCurrentLimit(40);
        leftMain.setInverted(true);
        leftMain.burnFlash();

        leftSecond.restoreFactoryDefaults();
        leftSecond.follow(leftMain);
        leftSecond.burnFlash();

        rightMain.restoreFactoryDefaults();
        rightMain.setSmartCurrentLimit(40);
        rightMain.setInverted(true);
        rightMain.burnFlash();

        rightSecond.restoreFactoryDefaults();
        rightSecond.follow(rightMain);
        rightSecond.burnFlash();

        // TODO: Measure track width
        kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(25));
    }

    public void drive(ChassisSpeeds speeds) {
        var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        wheelSpeeds.desaturate(DriveConstants.MAX_SPEED);

        double left = wheelSpeeds.leftMetersPerSecond / DriveConstants.MAX_SPEED;
        double right = wheelSpeeds.rightMetersPerSecond / DriveConstants.MAX_SPEED;

        SmartDashboard.putNumber("left", left);
        SmartDashboard.putNumber("right", right);

        leftMain.set(left);
        rightMain.set(right);
    }
}
