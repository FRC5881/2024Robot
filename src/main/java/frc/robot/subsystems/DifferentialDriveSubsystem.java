package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DifferentialDriveSubsystem extends SubsystemBase {
    private final DifferentialDriveKinematics kinematics;

    private final TalonSRX leftMain = new TalonSRX(1);
    private final TalonSRX leftSecond = new TalonSRX(2);

    private final TalonSRX rightMain = new TalonSRX(3);
    private final TalonSRX rightSecond = new TalonSRX(4);

    public DifferentialDriveSubsystem() {
        leftMain.setInverted(true);
        leftSecond.setInverted(true);

        leftSecond.follow(leftMain);
        rightSecond.follow(rightMain);

        // TODO: Measure track width
        kinematics = new DifferentialDriveKinematics(0.3);
    }

    public void drive(ChassisSpeeds speeds) {
        var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        wheelSpeeds.desaturate(DriveConstants.MAX_SPEED);

        double left = wheelSpeeds.leftMetersPerSecond / DriveConstants.MAX_SPEED;
        double right = wheelSpeeds.rightMetersPerSecond / DriveConstants.MAX_SPEED;

        leftMain.set(TalonSRXControlMode.PercentOutput, left);
        rightMain.set(TalonSRXControlMode.PercentOutput, right);
    }

}
