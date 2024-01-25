package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DifferentialDriveConstants;
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

        leftSecond.restoreFactoryDefaults();
        leftSecond.follow(leftMain);

        rightMain.restoreFactoryDefaults();
        rightMain.setSmartCurrentLimit(40);
        rightMain.setInverted(false);

        rightSecond.restoreFactoryDefaults();
        rightSecond.follow(rightMain);

        kinematics = new DifferentialDriveKinematics(DifferentialDriveConstants.TRACK_WIDTH);
    }

    /**
     * Drive the robot using a ChassisSpeeds object.
     * 
     * @param speeds
     */
    public void drive(ChassisSpeeds speeds) {
        var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        wheelSpeeds.desaturate(DifferentialDriveConstants.MAX_SPEED);

        double left = wheelSpeeds.leftMetersPerSecond / DifferentialDriveConstants.MAX_SPEED;
        double right = wheelSpeeds.rightMetersPerSecond / DifferentialDriveConstants.MAX_SPEED;

        leftMain.set(left);
        rightMain.set(right);
    }

    /**
     * Drive the robot using left and right values.
     * 
     * @param left  speed of left side in meters per second
     * @param right speed of right side in meters per second
     */
    public void drive(double left, double right) {
        leftMain.set(left / DifferentialDriveConstants.MAX_SPEED);
        rightMain.set(right / DifferentialDriveConstants.MAX_SPEED);
    }
}
