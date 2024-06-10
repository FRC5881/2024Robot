package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot.RobotFrame;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

import static edu.wpi.first.math.util.Units.feetToMeters;
import static edu.wpi.first.units.Units.Rotation;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive m_swerveDrive;
    private final Optional<GroundIntakeSubsystem> m_intake;

    /**
     * Creates a new SwerveSubsystem.
     * 
     * @param visionSubsystem The vision subsystem to use for pose estimation.
     * @throws IOException If the swerve module configuration file cannot be read.
     */
    public SwerveSubsystem(RobotFrame bot, Optional<GroundIntakeSubsystem> intake) throws IOException {
        this.m_intake = intake;

        String swerveDir;
        switch (bot) {
            case COMP:
                swerveDir = "compbot";
                break;
            case M1C2:
                swerveDir = "m1c2";
                break;
            default:
                throw new IOException("SwerveSubsystem is only configured for COMP and M1C2");
        }

        SwerveParser parser = new SwerveParser(new File(Filesystem.getDeployDirectory(), swerveDir));

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

    /**
     * Returns the current gyro angle.
     * 
     * @return The current gyro angle as a Rotation2d.
     */
    public Rotation2d getHeading() {
        // return m_swerveDrive.getYaw();
        return m_swerveDrive.getOdometryHeading();
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

    @Override
    public void periodic() {
        ChassisSpeeds speeds = m_swerveDrive.getRobotVelocity();
        SmartDashboard.putNumber("Swerve/Combined Speed",
                Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
    }

    private final PIDController visionPID = new PIDController(0.2, 0, 0.2 / 5.0);
    private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0.85 * 360, 720));

    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
    // This is 0.02 because the bot runs at 50hz (0.02 = 1/50), so one tick
    double kDt = 0.02;

    private Rotation2d m_last_heading = new Rotation2d();

    /// Uses a P-controller to minimize the supplier angle
    public Command cAutoPickUp(Supplier<Rotation2d> supplier) {
        cAutoPickUp(supplier, true);
    }

    public Command cAutoPickUp(Supplier<Rotation2d> supplier, boolean go) {
        SmartDashboard.putData("/Vision/Vision PID", visionPID);

        // visionPID.setTolerance(0.8);
        visionPID.enableContinuousInput(-180, 180);

        return runEnd(() -> {
            // measurement is the yaw returned from the Photonvision camera
            Rotation2d measurement = m_last_heading;
            m_last_heading = getHeading();

            SmartDashboard.putNumber("/Vision/Measurement", measurement.getDegrees());

            m_goal = new TrapezoidProfile.State(supplier.get().getDegrees() + measurement.getDegrees(), 0);
            SmartDashboard.putNumber("/Vision/Goal", m_goal.position);

            Rotation2d m_rot2d_goal = new Rotation2d(m_goal.position);
            Rotation2d error = getHeading().minus(m_rot2d_goal);

            m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);
            SmartDashboard.putNumber("/Vision/Setpoint", m_setpoint.position);

            double omega = visionPID.calculate(measurement.getDegrees(), m_setpoint.position);
            double rev = -1 * Math.cos(error.getRadians() * (90 / 35) * (Math.PI / 180));

            ChassisSpeeds speed;
            if (go == false) {
                speed = new ChassisSpeeds(0, 0, omega);
            } else {
                speed = new ChassisSpeeds(rev, 0, omega);
            }
            SmartDashboard.putNumber("/Vision/Turn Rate", omega);

            SmartDashboard.putBoolean("/Vision/Has Target", Vision.getInstance().hasTarget());
            if (!Vision.getInstance().hasTarget()) {
                stop();
                return;
            }

            driveRobotRelative(speed);
        }, this::stop).beforeStarting(() -> {
            m_last_heading = getHeading();
        }, this);
    }

    public Command cFollow() {
        double notepitch = Vision.getInstance().getTargetPitch();
        if (notepitch >= 60 && notepitch < 90) {
            return cAutoPickUp(Vision.getInstance()::getSmoothYaw, false);
        } else {
            return cAutoPickUp(Vision.getInstance()::getSmoothYaw);
        }
    }

    /// Drives forward forever
    public Command cDriveReverse() {
        return runEnd(() -> {
            ChassisSpeeds speed = new ChassisSpeeds(-0.5, 0, 0);
            driveRobotRelative(speed);
        }, this::stop);
    }

    public Command cTestTracking() {
        return cAutoPickUp(Vision.getInstance()::getSmoothYaw);
    }

    public Command cDumbSkedaddle() {
        if (m_intake.isEmpty()) {
            return Commands.print("Missing GroundIntake");
        }

        GroundIntakeSubsystem intake = m_intake.get();
        return Commands.race(
                cAutoPickUp(Vision.getInstance()::getSmoothYaw),
                intake.cRunUntilCaptured());
    }

    public double distTraveled() {
        // This will be a method to see the change in distance.
        // This will be checked every one second or so.
        double sum = 0;
        double iter = 0;

        Pose2d pose = getPose();

        return sum;
    }

    public Command cSmartSkedadle() {

        if (m_intake.isEmpty()) {
            return Commands.print("Missing GroundIntake");
        }

        GroundIntakeSubsystem intake = m_intake.get();

        // return Commands.sequence(
        // Commands.parallel(cTurnToTarget.getDegrees(Vision.getInstance()::getTargetYaw)),
        // Commands.race(Commands.waitSeconds(7.0), cDriveForward(),
        // intake.cRunUntilCaptured()));
        return Commands.none();
    }
}
