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
        io.setVoltages(0,0,0,0);
    }

    @Override
    public void periodic() {
        if (Robot.isSimulation()) {
            ((StraightShooterIOSim) io).update(0.02);
        }

        SmartDashboard.putNumber("/StraightShooter/Top Left Shooter RPM", io.getVelocityTL());
        SmartDashboard.putNumber("/StraightShooter/Top Right Shooter RPM", io.getVelocityTR());
        SmartDashboard.putNumber("/StraightShooter/Bottom Left Shooter RPM", io.getVelocityBL());
        SmartDashboard.putNumber("/StraightShooter/Bottom Right Shooter RPM", io.getVelocityBR());

        SmartDashboard.putNumber("/StraightShooter/FeedForward ks", io.getFFks());
        SmartDashboard.putNumber("/StraightShooter/FeedForward kv", io.getFFkv());
        SmartDashboard.putNumber("/StraightShooter/FeedForward ka", io.getFFka()); //
        ks = SmartDashboard.getNumber("/StraightShooter/FeedForward ks", io.getFFks());
        kv = SmartDashboard.getNumber("/StraightShooter/FeedForward kv", io.getFFkv());
        ka = SmartDashboard.getNumber("/StraightShooter/FeedForward ka", io.getFFka());

        SmartDashboard.putNumber("/StraightShooter/PID kp", io.getPIDkp());
        SmartDashboard.putNumber("/StraightShooter/PID ki", io.getPIDki());
        SmartDashboard.putNumber("/StraightShooter/PID kd", io.getPIDkd()); //
        kp = SmartDashboard.getNumber("/StraightShooter/PID kp", io.getPIDkp());
        ki = SmartDashboard.getNumber("/StraightShooter/PID ki", io.getPIDki());
        kd = SmartDashboard.getNumber("/StraightShooter/PID kd", io.getPIDkd());

        SmartDashboard.putNumber("/StraightShooter/VoltageDrive TL", io.getVoltageTL());
        SmartDashboard.putNumber("/StraightShooter/VoltageDrive TR", io.getVoltageTR());
        SmartDashboard.putNumber("/StraightShooter/VoltageDrive BL", io.getVoltageBL());
        SmartDashboard.putNumber("/StraightShooter/VoltageDrive BR", io.getVoltageBR()); //
        SmartDashboard.getNumber("/StraightShooter/VoltageDrive TL", io.getVoltageTL());
        SmartDashboard.getNumber("/StraightShooter/VoltageDrive TR", io.getVoltageTR());
        SmartDashboard.getNumber("/StraightShooter/VoltageDrive BL", io.getVoltageBL());
        SmartDashboard.getNumber("/StraightShooter/VoltageDrive BR", io.getVoltageBR());

        SmartDashboard.putNumber("/StraightShooter/VelocityDrive TL", io.getVelocityTL());
        SmartDashboard.putNumber("/StraightShooter/VelocityDrive TR", io.getVelocityTR());
        SmartDashboard.putNumber("/StraightShooter/VelocityDrive BL", io.getVelocityBL());
        SmartDashboard.putNumber("/StraightShooter/VelocityDrive BR", io.getVelocityBR()); //
        SmartDashboard.getNumber("/StraightShooter/VelocityDrive TL", io.getVelocityTL());
        SmartDashboard.getNumber("/StraightShooter/VelocityDrive TR", io.getVelocityTR());
        SmartDashboard.getNumber("/StraightShooter/VelocityDrive BL", io.getVelocityBL());
        SmartDashboard.getNumber("/StraightShooter/VelocityDrive BR", io.getVelocityBR());
    }

    // Drive all 4 motors at 12 volts
    public Command cRunAt12V() {
        SmartDashboard.putNumber("/StraightShooter/Shooter Voltage", 0);
        return runEnd(() -> {
            SmartDashboard.putNumber("/StraightShooter/Shooter Voltage", 12);
            io.setVoltages(12, 12, 12, 12);
        }, () -> {
            SmartDashboard.putNumber("/StraightShooter/Shooter Voltage", 0);
            io.setVoltages(0, 0, 0, 0);
        });
    }

    public Command cSetVelocities(double[] setpoints) {

        //setpoint in Rotations per second
        return runEnd(() -> {
            double TLSpeed = io.getVelocityTL();
            double TRSpeed = io.getVelocityTR();
            double BLSpeed = io.getVelocityBL();
            double BRSpeed = io.getVelocityBR();
            

            io.setVoltages(feedforwardTL.calculate(setpoints[0]) + pidControllerTL.calculate(TLSpeed, setpoints[0]),
                        feedforwardTR.calculate(setpoints[1]) + pidControllerTR.calculate(TRSpeed, setpoints[1]),
                        feedforwardBL.calculate(setpoints[2]) + pidControllerBL.calculate(BLSpeed, setpoints[2]),
                        feedforwardBR.calculate(setpoints[3]) + pidControllerBR.calculate(BRSpeed, setpoints[3])
                        );
        }, io::stop);
    }

    public Command cSetVelocities(double topLeft, double topRight, double bottomLeft, double bottomRight) {
        return cSetVelocities(topLeft, topRight, bottomLeft, bottomRight);
    }



}
