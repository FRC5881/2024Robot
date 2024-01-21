package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DifferentialDriveSubsystem extends SubsystemBase {
    private final DifferentialDriveKinematics kinematics;

    private final TalonSRX leftMain = new TalonSRX(1);
    private final TalonSRX leftSecond = new TalonSRX(2);

    private final TalonSRX rightMain = new TalonSRX(3);
    private final TalonSRX rightSecond = new TalonSRX(4);

    public DifferentialDriveSubsystem() {
        leftMain.setInverted(false);
        leftSecond.setInverted(false);

        rightMain.setInverted(true);
        rightSecond.setInverted(true);

        leftSecond.follow(leftMain);
        rightSecond.follow(rightMain);

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

        leftMain.set(TalonSRXControlMode.PercentOutput, left);
        rightMain.set(TalonSRXControlMode.PercentOutput, right);
    }
}
