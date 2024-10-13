package frc.robot.subsystems.StraightShooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class StraightShooterSubsystem extends SubsystemBase {
    //TODO: Check values --- if we're using different wheels we'll also have to have different controllers.
    double ks = 0;
    double kv = 0.12608;
    double ka = 0.0080106;
    
    private final SimpleMotorFeedforward feedforwardTL = new SimpleMotorFeedforward(0, 0.12608, 0.0080106);
    private final SimpleMotorFeedforward feedforwardTR = new SimpleMotorFeedforward(0, 0.12608, 0.0080106);
    private final SimpleMotorFeedforward feedforwardBL = new SimpleMotorFeedforward(0, 0.12608, 0.0080106);
    private final SimpleMotorFeedforward feedforwardBR = new SimpleMotorFeedforward(0, 0.12608, 0.0080106);
    
    double kp = 0.05;
    double ki = 0;
    double kd = 0;
    private final PIDController pidControllerTL = new PIDController(kp, ki, kd);
    private final PIDController pidControllerTR = new PIDController(kp, ki, kd);
    private final PIDController pidControllerBL = new PIDController(kp, ki, kd);
    private final PIDController pidControllerBR = new PIDController(kp, ki, kd);
    StraightShooterIO io;

    public StraightShooterSubsystem() {
        if (Robot.isReal()) {
            io = new StraightShooterIOReal();
        } else {
            io = new StraightShooterIOSim();
        }
    }

    @Override
    public void periodic() {
        if (Robot.isSimulation()) {
            ((StraightShooterIOSim) io).update(0.02);
        }

        SmartDashboard.putNumber("/StraightShooter/Top Left Shooter RPM", io.getTopLeftVelocity());
        SmartDashboard.putNumber("/StraightShooter/Top Right Shooter RPM", io.getTopRightVelocity());
        SmartDashboard.putNumber("/StraightShooter/Bottom Left Shooter RPM", io.getBottomLeftVelocity());
        SmartDashboard.putNumber("/StraightShooter/Bottom Right Shooter RPM", io.getBottomRightVelocity());
    }

    // Drive all 4 motors at 12 volts
    public Command cRunAt12V() {
        SmartDashboard.putNumber("/StraightShooter/Shooter Voltage", 0);
        return runEnd(() -> {
            SmartDashboard.putNumber("/StraightShooter/Shooter Voltage", 12);
            io.setVoltage(12, 12, 12, 12);
        }, () -> {
            SmartDashboard.putNumber("/StraightShooter/Shooter Voltage", 0);
            io.setVoltage(0, 0, 0, 0);
        });
    }

    public Command cSetVelocities(double[] setpoints) {
        SmartDashboard.putNumber("/StraightShooter/FeedForward ks", io.getFFks());
        SmartDashboard.putNumber("/StraightShooter/FeedForward kv", io.getFFkv());
        SmartDashboard.putNumber("/StraightShooter/FeedForward ka", io.getFFka());
        SmartDashboard.putNumber("/StraightShooter/PID kp", io.getPIDkp());
        SmartDashboard.putNumber("/StraightShooter/PID ki", io.getPIDki());
        SmartDashboard.putNumber("/StraightShooter/PID kd", io.getPIDkd());

        //setpoint in Rotations per second
        return runEnd(() -> {
            double TLSpeed = io.getTopLeftVelocity();
            double TRSpeed = io.getTopRightVelocity();
            double BLSpeed = io.getBottomLeftVelocity();
            double BRSpeed = io.getBottomRightVelocity();
            

            io.setVoltage(feedforwardTL.calculate(setpoints[0]) + pidControllerTL.calculate(TLSpeed, setpoints[0]),
                        feedforwardTR.calculate(setpoints[1]) + pidControllerTR.calculate(TRSpeed, setpoints[1]),
                        feedforwardBL.calculate(setpoints[2]) + pidControllerBL.calculate(BLSpeed, setpoints[2]),
                        feedforwardBR.calculate(setpoints[3]) + pidControllerBR.calculate(BRSpeed, setpoints[3])
                        );
        }, io::stop);
    }



}
