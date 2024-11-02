package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

import static edu.wpi.first.math.util.Units.feetToMeters;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive m_swerveDrive;

    /**
     * Creates a new SwerveSubsystem.
     * 
     * @param visionSubsystem The vision subsystem to use for pose estimation.
     * @throws IOException If the swerve module configuration file cannot be read.
     */
    public SwerveSubsystem() throws IOException {
        SwerveParser parser = new SwerveParser(new File(Filesystem.getDeployDirectory(), "compbot"));

        double maxSpeed = feetToMeters(12.5);
        m_swerveDrive = parser.createSwerveDrive(maxSpeed);

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetPose,
                this::getRobotRelativeSpeeds,
                this::driveRobotRelative,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5,
                        m_swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                        new ReplanningConfig()),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
                },
                this);

        SmartDashboard.setPersistent("/Pickup/Max Speed");
        SmartDashboard.setPersistent("/Pickup/Max Acceleration");
        SmartDashboard.setPersistent("/Pickup/Max Angular Speed");
        SmartDashboard.setPersistent("/Pickup/Max Angular Acceleration");
        SmartDashboard.setPersistent("/Pickup/X P");
        SmartDashboard.setPersistent("/Pickup/X I");
        SmartDashboard.setPersistent("/Pickup/X D");
        SmartDashboard.setPersistent("/Pickup/Y P");
        SmartDashboard.setPersistent("/Pickup/Y I");
        SmartDashboard.setPersistent("/Pickup/Y D");
        SmartDashboard.setPersistent("/Pickup/Omega P");
        SmartDashboard.setPersistent("/Pickup/Omega I");
        SmartDashboard.setPersistent("/Pickup/Omega D");
    }

    /**
     * Gets the current pose of the robot.
     * 
     * @return The estimated position (rotation + translation) of the robot.
     */
    public Pose2d getPose() {
        return m_swerveDrive.getPose();
    }

    /**
     * Resets the robot's position on the field.
     * 
     * @param pose The new position of the robot.
     */
    public void resetPose(Pose2d pose) {
        m_swerveDrive.resetOdometry(pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return m_swerveDrive.getRobotVelocity();
    }

    public void driveFieldRelative(ChassisSpeeds chassisSpeeds) {
        m_swerveDrive.driveFieldOriented(chassisSpeeds);
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        ChassisSpeeds realSpeed = new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond,
                -chassisSpeeds.omegaRadiansPerSecond);
        m_swerveDrive.drive(realSpeed);
    }

    // 3 PID controllers for the 3 components of the chassis speeds
    // as well as trapazodial motion profiling for (x,y) and omega
    private final PIDController m_xController = new PIDController(0.0, 0.0, 0.0);
    private TrapezoidProfile.State x_setpoint = new TrapezoidProfile.State(); 
   
    private final PIDController m_yController = new PIDController(0.0, 0.0, 0.0);
    private TrapezoidProfile.State y_setpoint = new TrapezoidProfile.State();

    private final PIDController m_omegaController = new PIDController(0.0, 0.0, 0.0);
    private TrapezoidProfile.State omega_setpoint = new TrapezoidProfile.State();

    private TrapezoidProfile m_translationProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(kPickupMaxSpeed, kPickupMaxAcceleration)
    );

    private TrapezoidProfile m_rotationProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(kPickupMaxSpeed, kPickupMaxAcceleration)
    );

    /**
     * Approaches a target position.
     * 
     * @param target dx, dy, and dtheta to the target position.
     * @return robot relative chassis speeds needed to approach the target.
     */
    public ChassisSpeeds driveTowards(Transform2d target) {
        Rotation2d angle = target.getRotation();

        TrapezoidProfile.State x_goal = new TrapezoidProfile.State(target.getX(), kPickupMaxSpeed);
        TrapezoidProfile.State y_goal = new TrapezoidProfile.State(target.getY(), kPickupMaxSpeed);
        TrapezoidProfile.State omega_goal = new TrapezoidProfile.State(angle.getRadians(), 0);

        x_setpoint = m_translationProfile.calculate(0.20, x_setpoint, x_goal);
        y_setpoint = m_translationProfile.calculate(0.20, y_setpoint, y_goal);
        omega_setpoint = m_rotationProfile.calculate(0.20, omega_setpoint, omega_goal);

        return new ChassisSpeeds(
            kPickupMaxSpeed * m_xController.calculate(x_setpoint.position),
            kPickupMaxSpeed * m_yController.calculate(y_setpoint.position),
            kPickupMaxAngularSpeed * m_omegaController.calculate(omega_setpoint.position)
        );
    }

    public Command cDriveTowards(Supplier<Transform2d> target) {
        return runEnd(() -> {
            driveRobotRelative(driveTowards(target.get()));
        }, this::stop);
    }

    /**
     * Returns the current gyro angle.
     * 
     * @return The current gyro angle as a Rotation2d.
     */
    public Rotation2d getHeading() {
        return m_swerveDrive.getYaw();
    }

    /**
     * Stops the swerve drive
     */
    public void stop() {
        m_swerveDrive.drive(new ChassisSpeeds());
    }

    /**
     * Zeros the gyroscope
     */
    public Command cZeroGyro() {
        return this.runOnce(() -> {
            m_swerveDrive.zeroGyro();
        });
    }

    public Command cLock() {
        return this.run(() -> m_swerveDrive.lockPose());
    }

    /**
     * Returns the maximum velocity of the swerve drive.
     * 
     * @return A measure of the maximum velocity of the swerve drive.
     */
    public Measure<Velocity<Distance>> getMaximumVelocity() {
        return Units.MetersPerSecond.of(m_swerveDrive.getMaximumVelocity());
    }

    /**
     * Returns the maximum angular velocity of the swerve drive.
     * 
     * @return A measure of the maximum angular velocity of the swerve drive.
     */
    public Measure<Velocity<Angle>> getMaximumAngularVelocity() {
        return Units.RadiansPerSecond.of(m_swerveDrive.getMaximumAngularVelocity());
    }

    public static double kPickupMaxSpeed = 3.0; // meters per second
    public static double kPickupMaxAcceleration = 3.0; // meters per second squared

    public static double kPickupMaxAngularSpeed = Math.PI; // radians per second
    public static double kPickupMaxAngularAcceleration = Math.PI; // radians per second squared
    
    @Override
    public void periodic() {
        // double new_max_speed = SmartDashboard.getNumber("/Pickup/Max Speed", kPickupMaxSpeed);
        // double new_max_acceleration = SmartDashboard.getNumber("/Pickup/Max Acceleration", kPickupMaxAcceleration);
        // if (new_max_speed != kPickupMaxSpeed || new_max_acceleration != kPickupMaxAcceleration) {
        //     kPickupMaxSpeed = new_max_speed;
        //     kPickupMaxAcceleration = new_max_acceleration;
        //     m_translationProfile = new TrapezoidProfile(
        //         new TrapezoidProfile.Constraints(kPickupMaxSpeed, kPickupMaxAcceleration)
        //     );
        // }

        // double new_max_angular_speed = SmartDashboard.getNumber("/Pickup/Max Angular Speed", kPickupMaxAngularSpeed);
        // double new_max_angular_acceleration = SmartDashboard.getNumber("/Pickup/Max Angular Acceleration", kPickupMaxAngularAcceleration);
        // if (new_max_angular_speed != kPickupMaxAngularSpeed || new_max_angular_acceleration != kPickupMaxAngularAcceleration) {
        //     kPickupMaxAngularSpeed = new_max_angular_speed;
        //     kPickupMaxAngularAcceleration = new_max_angular_acceleration;

        //     m_rotationProfile = new TrapezoidProfile(
        //         new TrapezoidProfile.Constraints(kPickupMaxSpeed, kPickupMaxAcceleration)
        //     );
        // }

        // m_xController.setP(SmartDashboard.getNumber("/Pickup/X P", m_xController.getP()));
        // m_xController.setI(SmartDashboard.getNumber("/Pickup/X I", m_xController.getI()));
        // m_xController.setD(SmartDashboard.getNumber("/Pickup/X D", m_xController.getD()));

        // m_yController.setP(SmartDashboard.getNumber("/Pickup/Y P", m_yController.getP()));
        // m_yController.setI(SmartDashboard.getNumber("/Pickup/Y I", m_yController.getI()));
        // m_yController.setD(SmartDashboard.getNumber("/Pickup/Y D", m_yController.getD()));

        // m_omegaController.setP(SmartDashboard.getNumber("/Pickup/Omega P", m_omegaController.getP()));
        // m_omegaController.setI(SmartDashboard.getNumber("/Pickup/Omega I", m_omegaController.getI()));
        // m_omegaController.setD(SmartDashboard.getNumber("/Pickup/Omega D", m_omegaController.getD()));
    }
}
