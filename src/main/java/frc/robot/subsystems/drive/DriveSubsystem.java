package frc.robot.subsystems.drive;

import java.util.Arrays;
import java.util.NoSuchElementException;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
// for pose est.
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.pathplanning.LocalADStarAK;
import frc.lib.swerve.SwerveAlgorithms;
import frc.lib.sysid.SwerveDriveSysidRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.PivotId;
import frc.robot.Constants.SwerveDriveDimensions;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.sensors.Gyro.Gyro;
import frc.robot.sensors.Vision.AprilTagVision;
import frc.robot.subsystems.drive.Module.Module;
import frc.robot.subsystems.drive.Module.ModuleIO;
import frc.robot.subsystems.drive.Module.ModuleIOSim;
import frc.robot.subsystems.drive.Module.ModuleIOSparkMax;

public class DriveSubsystem extends SubsystemBase {

    // DRIVE COMMAND VARIABLES:
    JoystickCleaner joystickCleaner = new JoystickCleaner();

    final double SLOW_LINEAR_SPEED = 0.5;
    final double SLOW_ROTATIONAL_SPEED = 0.3;

    final double LOWER_DB = 0.15;
    final double UPPER_DB = 0.15;

    final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
    boolean fieldRelative = true;

    double iXSpeed;
    double iYSpeed;
    double angle;
    double offset;

    double xSpeed;
    double ySpeed;
    double rot;

    boolean hold = false;


    Gyro gyro;
    AprilTagVision vision;

    private Module frontLeft;
    private Module frontRight;
    private Module backLeft;
    private Module backRight;

    private final double maxSpeed = Constants.SwerveDriveDimensions.maxSpeed;

    private SwerveModuleState[] desiredSwerveStates = new SwerveModuleState[] {};

    // SwerveDriveOdometry odometry;
    SwerveDrivePoseEstimator swervePoseEstimator; // swerve pose estimator is an alt. for swerve odometry
    SysIdRoutine sysIdRoutine;
    Pose2d odometryPose = new Pose2d();

    public DriveSubsystem(Gyro gyro, AprilTagVision vision) {
        this.gyro = gyro;
        this.vision = vision;
        switch (Robot.getMode()) {
            case REAL:
                frontLeft = new Module(new ModuleIOSparkMax(ModuleConstants.FL), PivotId.FL);
                frontRight = new Module(new ModuleIOSparkMax(ModuleConstants.FR), PivotId.FR);
                backLeft = new Module(new ModuleIOSparkMax(ModuleConstants.BL), PivotId.BL);
                backRight = new Module(new ModuleIOSparkMax(ModuleConstants.BR), PivotId.BR);
                break;

            case SIM:
                frontLeft = new Module(new ModuleIOSim(), PivotId.FL);
                frontRight = new Module(new ModuleIOSim(), PivotId.FR);
                backLeft = new Module(new ModuleIOSim(), PivotId.BL);
                backRight = new Module(new ModuleIOSim(), PivotId.BR);
                break;

            default:
                frontLeft = new Module(new ModuleIO() {
                }, PivotId.FL);
                frontRight = new Module(new ModuleIO() {
                }, PivotId.FR);
                backLeft = new Module(new ModuleIO() {
                }, PivotId.BL);
                backRight = new Module(new ModuleIO() {
                }, PivotId.BR);
                break;
        }
        // create sysidroutine
        sysIdRoutine = new SwerveDriveSysidRoutine().createNewRoutine(frontLeft, frontRight, backLeft, backRight, this,
                new SysIdRoutine.Config());

        // Create Pose Estimator
        // odometry = new SwerveDriveOdometry(SwerveDriveDimensions.kinematics,
        // gyro.getAngleRotation2d(), getModulePositionsArray());
        swervePoseEstimator = new SwerveDrivePoseEstimator(
                SwerveDriveDimensions.kinematics,
                gyro.getAngleRotation2d(),
                getModulePositionsArray(),
                new Pose2d(),
                VecBuilder.fill(0.05, 0.05, 0.05),
                VecBuilder.fill(VisionConstants.xyStdDev, VisionConstants.xyStdDev, VisionConstants.thetaStdDev));// THIS
                                                                                                                  // IS
                                                                                                                  // SUPPOSED
                                                                                                                  // TO
                                                                                                                  // BE
                                                                                                                  // THE
                                                                                                                  // starting
                                                                                                                  // standard
                                                                                                                  // deviations

        // Configure pathplanner
        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetOdometry,
                () -> SwerveDriveDimensions.kinematics.toChassisSpeeds(getActualSwerveStates()),
                this::driveChassisSpeedsNoScaling,
                new HolonomicPathFollowerConfig(
                        SwerveDriveDimensions.maxSpeed,
                        SwerveAlgorithms.computeMaxNorm(SwerveDriveDimensions.positions, new Translation2d(0, 0)),
                        new ReplanningConfig()),
                () -> DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red,
                this);
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    Logger.recordOutput(
                            "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
                });
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> {
                    Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
                });
    }

    @Override
    public void periodic() {
        // Update modules
        frontLeft.periodic();
        frontRight.periodic();
        backLeft.periodic();
        backRight.periodic();
        // Update odometry
        updateOdometry();
        // Log odometry
        Logger.recordOutput("Drive/OdometryPosition", odometryPose);
        // Log swerve states
        Logger.recordOutput("Drive/SwerveStates", getActualSwerveStates());
        Logger.recordOutput("Drive/DesiredSwerveStates", getDesiredSwerveStates());
    }

    public SwerveModuleState[] getActualSwerveStates() {
        return new SwerveModuleState[] { frontLeft.getState(), frontRight.getState(), backLeft.getState(),
                backRight.getState() };
    }

    public SwerveModuleState[] getDesiredSwerveStates() {
        return desiredSwerveStates;
    }

    private void updateOdometry() {
        if (vision.isTarget()) {

            double distConst = Math.pow(vision.getDistance(), 2.0);
            swervePoseEstimator.addVisionMeasurement(vision.getAprilTagPose2d(), vision.getLatency(),
                    VecBuilder.fill(VisionConstants.xyStdDev * distConst,
                            VisionConstants.xyStdDev * distConst, VisionConstants.thetaStdDev * distConst));
        }
        odometryPose = swervePoseEstimator.update(gyro.getRawAngleRotation2d(), getModulePositionsArray());

    }

    private void resetOdometry(Pose2d newPose) {
        swervePoseEstimator.resetPosition(gyro.getRawAngleRotation2d(), getModulePositionsArray(), newPose);
        odometryPose = newPose;
    }

    public Pose2d getPose() {
        return odometryPose;
    }

    public SwerveModulePosition[] getModulePositionsArray() {
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition() };
    }

    /**
     * Method to drive the robot using joystick info and desaturated swerve
     * algorithm.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    private void drivePercentDesaturated(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

        xSpeed = xSpeed * maxSpeed;
        ySpeed = ySpeed * maxSpeed;
        rot = rot * maxSpeed
                / SwerveAlgorithms.computeMaxNorm(SwerveDriveDimensions.positions, new Translation2d(0, 0));

        SwerveModuleState[] swerveModuleStates = SwerveAlgorithms.desaturated(xSpeed, ySpeed, rot,
                gyro.getAngleRotation2d().getRadians(), fieldRelative);

        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
        desiredSwerveStates = swerveModuleStates;
    }

    /**
     * Method to drive the robot using joystick info and double cone swerve
     * algorithm.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    private void drivePercentDoubleCone(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        xSpeed = xSpeed * maxSpeed;
        ySpeed = ySpeed * maxSpeed;
        rot = rot * maxSpeed
                / SwerveAlgorithms.computeMaxNorm(SwerveDriveDimensions.positions, new Translation2d(0, 0));
        SwerveModuleState[] swerveModuleStates = SwerveAlgorithms.doubleCone(xSpeed, ySpeed, rot,
                gyro.getAngleRotation2d().getRadians(), fieldRelative);
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
        desiredSwerveStates = swerveModuleStates;
    }

    private void drivePercentDoubleCone(double xSpeed, double ySpeed, double rot, boolean fieldRelative,
            Translation2d centerOfRotation) {
        xSpeed = xSpeed * maxSpeed;
        ySpeed = ySpeed * maxSpeed;
        rot = rot * maxSpeed
                / SwerveAlgorithms.computeMaxNorm(SwerveDriveDimensions.positions, new Translation2d(0, 0));
        SwerveModuleState[] swerveModuleStates = SwerveAlgorithms.doubleCone(xSpeed, ySpeed, rot,
                gyro.getAngleRotation2d().getRadians(), fieldRelative, centerOfRotation);
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
        desiredSwerveStates = swerveModuleStates;
    }

    private void driveChassisSpeedsNoScaling(ChassisSpeeds speeds) {
        SwerveModuleState[] swerveModuleStates = SwerveAlgorithms.rawSpeeds(speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
        desiredSwerveStates = swerveModuleStates;
    }

    // Commands
    public Command resetGyroCommand() {
        Command c = new InstantCommand(() -> {
            gyro.reset();
            resetOdometry(getPose());
        });
        c.addRequirements(this);
        return c;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public Command resetOdometryCommand(Pose2d newPose) {
        Command c = new InstantCommand(() -> resetOdometry(newPose));
        c.addRequirements(this);
        return c;
    }

    public Command joystickDriveCommand(CommandXboxController driverController) {
        Command c = new Command() {
            @Override
            public void execute() {
                driverController.back().onTrue(new InstantCommand(() -> fieldRelative = !fieldRelative));

                Trigger leftTrigger = new Trigger(() -> driverController.getLeftTriggerAxis() > 0.1);

                Trigger rightTrigger = new Trigger(() -> driverController.getRightTriggerAxis() > 0.1);

                if (!rightTrigger.getAsBoolean()) {
                    xSpeed = -driverController.getLeftY() * SLOW_LINEAR_SPEED;
                    ySpeed = -driverController.getLeftX() * SLOW_LINEAR_SPEED;
                    rot = -driverController.getRightX() * SLOW_ROTATIONAL_SPEED;
                } else {
                    xSpeed = -m_xspeedLimiter.calculate(driverController.getLeftY());
                    ySpeed = -m_yspeedLimiter.calculate(driverController.getLeftX());
                    rot = -m_rotLimiter.calculate(driverController.getRightX());
                }

                /* Apply linear deadband */
                joystickCleaner.setX(xSpeed);
                joystickCleaner.setY(ySpeed);
                joystickCleaner.applyDeadband(LOWER_DB, UPPER_DB);
                xSpeed = joystickCleaner.getX();
                ySpeed = joystickCleaner.getY();

                /* Apply rotational deadband */
                rot = MathUtil.applyDeadband(rot, LOWER_DB);

                /* Increase rotational sensitivity */
                rot = Math.signum(rot) * Math.pow(Math.abs(rot), 1.0 / 3.0);

                if (!hold && leftTrigger.getAsBoolean()) {
                    iXSpeed = xSpeed;
                    iYSpeed = ySpeed;
                    offset = -gyro.getAngleRotation2d().getRadians();
                    hold = true;
                }
                if (!leftTrigger.getAsBoolean()) {
                    hold = false;
                }
                angle = (Math.atan2(iYSpeed, iXSpeed) + offset); // add gyro offset to angle
                if (leftTrigger.getAsBoolean()) { // drive with center of rot around closest pivot
                    Translation2d a = Arrays.stream(SwerveDriveDimensions.positions)
                            .reduce((best,
                                    current) -> Math.abs((SwerveAlgorithms
                                            .angleDistance(Math.atan2(current.getY(), current.getX()), angle))) < Math
                                                    .abs((SwerveAlgorithms.angleDistance(
                                                            Math.atan2(best.getY(), best.getX()), angle)))
                                                                    ? current
                                                                    : best)
                            .orElseThrow(() -> new NoSuchElementException("No closest pivot."));
                    drivePercentDoubleCone(xSpeed, ySpeed, rot, fieldRelative, a);
                    Logger.recordOutput("Drive/CoR", a);
                } else {
                    drivePercentDoubleCone(xSpeed, ySpeed, rot, fieldRelative);
                }
            }

            @Override
            public void end(boolean interrupted) {
                drivePercentDoubleCone(0, 0, 0, false);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        c.addRequirements(this);
        return c;
    }

}
