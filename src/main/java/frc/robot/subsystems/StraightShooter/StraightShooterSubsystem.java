package frc.robot.subsystems.StraightShooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

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

        SmartDashboard.putNumber("/StraightShooter/PID kp", kp);
        SmartDashboard.putNumber("/StraightShooter/PID ki", ki);
        SmartDashboard.putNumber("/StraightShooter/PID kd", kd);

        SmartDashboard.putNumber("/StraightShooter/VelocityDrive TL", 0);
        SmartDashboard.putNumber("/StraightShooter/VelocityDrive TR", 0);
        SmartDashboard.putNumber("/StraightShooter/VelocityDrive BL", 0);
        SmartDashboard.putNumber("/StraightShooter/VelocityDrive BR", 0);
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

        double newKp = SmartDashboard.getNumber("/StraightShooter/PID kp", kp);
        double newKi = SmartDashboard.getNumber("/StraightShooter/PID ki", ki);
        double newKd = SmartDashboard.getNumber("/StraightShooter/PID kd", kd);

        if (newKp != pidControllerTL.getP()) {
            pidControllerTL.setP(newKp);
            pidControllerTR.setP(newKp);
            pidControllerBL.setP(newKp);
            pidControllerBR.setP(newKp);
        }
        if (newKi != pidControllerTL.getI()) {
            pidControllerTL.setI(newKi);
            pidControllerTR.setI(newKi);
            pidControllerBL.setI(newKi);
            pidControllerBR.setI(newKi);
        }
        if (newKd != pidControllerTL.getD()) {
            pidControllerTL.setD(newKd);
            pidControllerTR.setD(newKd);
            pidControllerBL.setD(newKd);
            pidControllerBR.setD(newKd);
        }
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

    public void voltageCalculator(double[] setpoints) {
        SmartDashboard.putNumber("/StraightShooter/Shooter BL Setpoint", setpoints[2]);

        double TLSpeed = io.getVelocityTL();
        double TRSpeed = io.getVelocityTR();
        double BLSpeed = io.getVelocityBL();
        double BRSpeed = io.getVelocityBR();

        io.setVoltages(feedforwardTL.calculate(setpoints[0]) + pidControllerTL.calculate(TLSpeed, setpoints[0]),
                    feedforwardTR.calculate(setpoints[1]) + pidControllerTR.calculate(TRSpeed, setpoints[1]),
                    feedforwardBL.calculate(setpoints[2]) + pidControllerBL.calculate(BLSpeed, setpoints[2]),
                    feedforwardBR.calculate(setpoints[3]) + pidControllerBR.calculate(BRSpeed, setpoints[3])
                    );
    }

    
    public Command cSetConstantVelocities(double[] setpoints) {
        return runEnd(() -> voltageCalculator(setpoints), io::stop);
    }

    public Command cSetConstantVelocities(double topLeft, double topRight, double bottomLeft, double bottomRight) {
        double[] velocities = {topLeft, topRight, bottomLeft, bottomRight};
        return cSetConstantVelocities(velocities);
    }

    public Command cSetSmartDashboardVelocities() {
        return runEnd(() -> {
            double TLSetpoint = SmartDashboard.getNumber("/StraightShooter/VelocityDrive TL", 0);
            double TRSetpoint = SmartDashboard.getNumber("/StraightShooter/VelocityDrive TR", 0);
            double BLSetpoint = SmartDashboard.getNumber("/StraightShooter/VelocityDrive BL", 0);
            double BRSetpoint = SmartDashboard.getNumber("/StraightShooter/VelocityDrive BR", 0);
    
            voltageCalculator(new double[] {
                TLSetpoint, TRSetpoint, BLSetpoint, BRSetpoint
            });
        }, io::stop);
    }

    // voltage
    // position
    // velocity
    private final MutableMeasure<Voltage> volts = MutableMeasure.mutable(Units.Volts.of(0));
    private final MutableMeasure<Angle> rotations = MutableMeasure.mutable(Units.Rotations.of(0));
    private final MutableMeasure<Velocity<Angle>> velocity = MutableMeasure.mutable(Units.Rotations.of(0).per(Units.Minute.of(1)));

    private String motorName = "TopLeft";
    private int motorId = 0;

    private void setVoltage(Measure<Voltage> v) {
        io.setVoltages(v.baseUnitMagnitude(), v.baseUnitMagnitude(), v.baseUnitMagnitude(), v.baseUnitMagnitude());

        volts.mut_setBaseUnitMagnitude(v.baseUnitMagnitude());
        velocity.mut_setMagnitude(io.getVelocities()[motorId]);
        rotations.mut_setMagnitude(io.getVelocities()[motorId]);
    }

    private void log(SysIdRoutineLog log) {
        log.motor(motorName)
            .voltage(volts)
            .angularPosition(rotations)
            .angularVelocity(velocity);
    }

    private SysIdRoutine.Config config = new SysIdRoutine.Config(Units.Volts.of(0.5).per(Units.Seconds.of(1)),
                                                                Units.Volts.of(10.5),
                                                                Units.Seconds.of(12 / 0.5));
    private SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(this::setVoltage, this::log, this);
    private SysIdRoutine routine = new SysIdRoutine(config, mechanism);

    public Command cSysid() {
        return 
            (
                routine.dynamic(Direction.kForward)
                .withTimeout(5)
            ).andThen(
                routine.dynamic(Direction.kReverse)
                .withTimeout(5)
            )
            .andThen(Commands.waitSeconds(5))
            .andThen(routine.quasistatic(Direction.kForward))

            .andThen(Commands.waitSeconds(5))
            .andThen(routine.quasistatic(Direction.kReverse));
    }
}
