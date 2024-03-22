package frc.lib.drive;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
// for pose est.
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.drive.Module.Module;
import frc.lib.drive.Module.ModuleIO;
import frc.lib.drive.Module.ModuleIOSim;
import frc.lib.drive.Module.ModuleIOSparkMax;
import frc.lib.pathplanning.LocalADStarAK;
import frc.lib.swerve.SwerveAlgorithms;
import frc.lib.sysid.SwerveDriveSysidRoutine;
import frc.robot.Constants;
import frc.robot.DashboardInit;
import frc.robot.Constants.AprilTagVisionConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.PivotId;
import frc.robot.Constants.SwerveDriveDimensions;
import frc.robot.Robot;
import frc.robot.sensors.Gyro.Gyro;
import frc.robot.sensors.Vision.AprilTagVision.AprilTagVision;
import frc.robot.subsystems.drive.DriveWeightCommand;

public class DriveSubsystem extends SubsystemBase {

    Gyro gyro;
    ArrayList<AprilTagVision> visions = new ArrayList<AprilTagVision>();

    private Module frontLeft;
    private Module frontRight;
    private Module backLeft;
    private Module backRight;
    boolean usedAprilTag;

    PIDController pidr = frc.robot.Constants.PIDConstants.constructPID(frc.robot.Constants.PIDConstants.rotPID,
            "RotateToAngle1");

    private final double maxSpeed = Constants.SwerveDriveDimensions.maxSpeed;

    private SwerveModuleState[] desiredSwerveStates = new SwerveModuleState[] {};

    double dynamicThreshold = 0.8;

    // SwerveDriveOdometry odometry;
    SwerveDrivePoseEstimator swervePoseEstimator;
    SysIdRoutine sysIdRoutine;
    Pose2d odometryPose = new Pose2d();

    public DriveSubsystem(Gyro gyro, ArrayList<AprilTagVision> visions) {
        this.gyro = gyro;
        this.visions = visions;
        switch (Robot.getMode()) { // create modules
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
        swervePoseEstimator = new SwerveDrivePoseEstimator(
                SwerveDriveDimensions.kinematics,
                gyro.getAngleRotation2d(),
                getModulePositionsArray(),
                new Pose2d(),
                VecBuilder.fill(0.6, 0.6, 0.001),
                VecBuilder.fill(3.5, 3.5, 9999999));

        // Configure pathplanner
        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetOdometryAuton,
                () -> SwerveDriveDimensions.kinematics.toChassisSpeeds(getActualSwerveStates()),
                this::driveChassisSpeedsDesaturated,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(3, 0, 0),
                        new PIDConstants(3.5, 0, 0),
                        SwerveDriveDimensions.maxSpeed,
                        SwerveAlgorithms.computeMaxNorm(SwerveDriveDimensions.positions, new Translation2d(0, 0)),
                        new ReplanningConfig()),
                () -> DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red,
                this);
        // setup pathplanning logs
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    Logger.recordOutput(
                            "Drive/Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
                });
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> {
                    Logger.recordOutput("Drive/Odometry/TrajectorySetpoint", targetPose);
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
        Logger.recordOutput("Drive/Odometry/OdometryPosition", odometryPose);
        // Log swerve states
        Logger.recordOutput("Drive/SwerveStates", getActualSwerveStates());
        Logger.recordOutput("Drive/DesiredSwerveStates", getDesiredSwerveStates());

        DashboardInit.setFieldPos(getPose());
    }

    public SwerveModuleState[] getActualSwerveStates() {
        return new SwerveModuleState[] { frontLeft.getState(), frontRight.getState(), backLeft.getState(),
                backRight.getState() };
    }

    public SwerveModuleState[] getDesiredSwerveStates() {
        return desiredSwerveStates;
    }

    private void updateOdometry() {
        for (AprilTagVision vision : visions) {
            if (vision.isTarget() && vision.isPoseValid(vision.getAprilTagPose2d())
                    && vision.getNumVisibleTags() != 0 && Robot.inTeleop) {

                double distConst = 1 + (vision.getDistance() * vision.getDistance()); // distance standard deviation
                                                                                      // constant
                double poseDifference = vision.getAprilTagPose2d().getTranslation()
                        .getDistance(getPose().getTranslation());
                double poseDifferenceTheta = Math.abs(Math.toDegrees(SwerveAlgorithms.angleDistance(
                        vision.getAprilTagPose2d().getRotation().getRadians(), getPose().getRotation().getRadians())));
                double poseDifferenceDeviation = 1 / (1 + poseDifference);

                double posDifX = vision.getAprilTagPose2d().getTranslation().getX() - getPose().getX();
                double posDifY = vision.getAprilTagPose2d().getTranslation().getY() - getPose().getY();

                double xy = 0;
                double theta = 0;

                if (Robot.inTeleop) {
                    xy = AprilTagVisionConstants.xyStdDev;
                    theta = AprilTagVisionConstants.thetaStdDev;
                } else {
                    xy = AprilTagVisionConstants.xyStdDevAuto;
                    theta = AprilTagVisionConstants.thetaStdDevAuto;
                }
                boolean useEstimate = true;

                double speed = Math.hypot(SwerveDriveDimensions.kinematics.toChassisSpeeds(
                        getActualSwerveStates()).vxMetersPerSecond,
                        SwerveDriveDimensions.kinematics.toChassisSpeeds(getActualSwerveStates()).vyMetersPerSecond);

                if (speed > 0.3) {
                    dynamicThreshold += (speed - 0.3) * 0.02;
                    dynamicThreshold = Math.min(1.5, dynamicThreshold);
                }

                if (poseDifference > dynamicThreshold) {
                    useEstimate = false;
                }

                if (vision.getNumVisibleTags() > 1) {
                    useEstimate = true;
                    // if (Robot.inTeleop) {
                        
                    // } else {
                    //     xy = AprilTagVisionConstants.xyStdDev;
                    // }

                    xy = 0.5;
                    // distConst = distConst
                } else {
                    distConst = distConst * 2;
                }

                Logger.recordOutput("PosDifference", poseDifference);
                Logger.recordOutput("PosDifX", posDifX);
                Logger.recordOutput("PosDifY", posDifY);
                Logger.recordOutput("PosDifTheta", poseDifferenceTheta);
                Logger.recordOutput("DynamicThreshold", dynamicThreshold);
                if (useEstimate) {
                    usedAprilTag = true;
                    dynamicThreshold -= (0.2 * 0.02 * vision.getNumVisibleTags()) / (distConst / 2);
                    dynamicThreshold = Math.max(dynamicThreshold, 0.4);
                    swervePoseEstimator.addVisionMeasurement(vision.getAprilTagPose2d(), vision.getLatency(),
                            VecBuilder.fill(xy * distConst,
                                    xy * distConst,
                                    Math.toRadians(theta) * distConst));
                }

                Logger.recordOutput("DynamicThreshold", dynamicThreshold);
            }

        }
        // update odometry
        odometryPose = swervePoseEstimator.updateWithTime(Timer.getFPGATimestamp(), gyro.getRawAngleRotation2d(),
                getModulePositionsArray());

    }

    private void resetOdometry(Pose2d newPose) {
        swervePoseEstimator.resetPosition(gyro.getRawAngleRotation2d(), getModulePositionsArray(), newPose);
        odometryPose = newPose;
    }

    private void resetOdometryAuton(Pose2d pose) {
        // if (pose.getTranslation().getDistance(getPose().getTranslation()) < 1.5){
        // resetOdometry(getPose());
        // }
        // else{
        // resetOdometry(pose);
        // }

        gyro.setOffset(gyro.getRawAngleRadians() - pose.getRotation().getRadians() +
                (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red ? Math.PI
                        : 0));
        resetOdometry(pose);
        // gyro.setOffset(gyro.getRawAngleRadians() - pose.getRotation().getRadians());

    }

    public Command rotateToAngleCommand(DoubleSupplier angleSupplier) {
        Command c = new Command() {
            double angle;
            @Override
            public void initialize() {
                angle = angleSupplier.getAsDouble();
            }

            @Override
            public void end(boolean interrupted) {
                System.out.println("end");
                driveDoubleConePercent(0, 0, 0, false, new Translation2d());
            }

            @Override
            public void execute() {
                System.out.println(Math.toDegrees(angle));
                
                double o;
                o = pidr.calculate(-SwerveAlgorithms.angleDistance(gyro.getAngleRotation2d().getRadians(),
                        angle), 0);
                if (Math.abs(o) < 0.005) {
                    o = 0;
                }
                o = MathUtil.clamp(o, -1, 1);

                driveDoubleConePercent(0, 0, o, false, new Translation2d());

            }

            @Override
            public boolean isFinished() {
                return (Math.abs(SwerveAlgorithms.angleDistance(gyro.getAngleRotation2d().getRadians(),
                        angle)) < 3);
            }
        };
        c.addRequirements(this);
        return c;
    }

    public ChassisSpeeds getChassisSpeeds() {
        Logger.recordOutput("Drive/Actual Chassis Speeds",
                SwerveDriveDimensions.kinematics.toChassisSpeeds(getActualSwerveStates()));
        return SwerveDriveDimensions.kinematics.toChassisSpeeds(getActualSwerveStates());
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
     * Method to drive the robot using joystick info and desaturated
     * algorithm.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */

    private void driveDesaturated(double xSpeed, double ySpeed, double rot, boolean fieldRelative,
            Translation2d centerOfRot) {

        xSpeed = xSpeed * maxSpeed;
        ySpeed = ySpeed * maxSpeed;
        rot = rot * maxSpeed
                / SwerveAlgorithms.computeMaxNorm(SwerveDriveDimensions.positions, new Translation2d(0, 0));

        SwerveModuleState[] swerveModuleStates = SwerveAlgorithms.desaturated(xSpeed, ySpeed, rot,
                gyro.getAngleRotation2d().getRadians(), fieldRelative, centerOfRot);

        frontLeft.setDesiredStateMetersPerSecond(swerveModuleStates[0]);
        frontRight.setDesiredStateMetersPerSecond(swerveModuleStates[1]);
        backLeft.setDesiredStateMetersPerSecond(swerveModuleStates[2]);
        backRight.setDesiredStateMetersPerSecond(swerveModuleStates[3]);
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

    private void driveDoubleCone(double xSpeed, double ySpeed, double rot, boolean fieldRelative,
            Translation2d centerOfRotation) {
        rot = rot
                / SwerveAlgorithms.computeMaxNorm(SwerveDriveDimensions.positions, centerOfRotation);
        SwerveModuleState[] swerveModuleStates = SwerveAlgorithms.doubleCone(xSpeed, ySpeed, rot,
                gyro.getAngleRotation2d().getRadians(), fieldRelative, centerOfRotation);
        frontLeft.setDesiredStateMetersPerSecond(swerveModuleStates[0]);
        frontRight.setDesiredStateMetersPerSecond(swerveModuleStates[1]);
        backLeft.setDesiredStateMetersPerSecond(swerveModuleStates[2]);
        backRight.setDesiredStateMetersPerSecond(swerveModuleStates[3]);
        desiredSwerveStates = swerveModuleStates;
    }

    private void driveDoubleConePercent(double xSpeed, double ySpeed, double rot, boolean fieldRelative,
            Translation2d centerOfRotation) {
        xSpeed = xSpeed * maxSpeed;
        ySpeed = ySpeed * maxSpeed;
        rot = rot * maxSpeed
                / SwerveAlgorithms.computeMaxNorm(SwerveDriveDimensions.positions, centerOfRotation);
        SwerveModuleState[] swerveModuleStates = SwerveAlgorithms.doubleCone(xSpeed, ySpeed, rot,
                gyro.getAngleRotation2d().getRadians(), fieldRelative, centerOfRotation);
        
        frontRight.setDesiredStateMetersPerSecond(swerveModuleStates[1]);
        frontLeft.setDesiredStateMetersPerSecond(swerveModuleStates[0]);
        backLeft.setDesiredStateMetersPerSecond(swerveModuleStates[2]);
        backRight.setDesiredStateMetersPerSecond(swerveModuleStates[3]);
        desiredSwerveStates = swerveModuleStates;
    }

    /**
     * Method to drive the robot using chassis speeds and desaturated algorithm.
     */
    private void driveChassisSpeedsDesaturated(ChassisSpeeds speeds) {
        SwerveModuleState[] swerveModuleStates = SwerveAlgorithms.desaturated(speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, 0, false);
        frontLeft.setDesiredStateMetersPerSecond(swerveModuleStates[0]);
        frontRight.setDesiredStateMetersPerSecond(swerveModuleStates[1]);
        backLeft.setDesiredStateMetersPerSecond(swerveModuleStates[2]);
        backRight.setDesiredStateMetersPerSecond(swerveModuleStates[3]);
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
    // public Command resetGyroCommand(double angle) {
    // Command c = new InstantCommand(() -> {

    // resetOdometry(getPose());
    // });
    // c.addRequirements(this);
    // return c;
    // }

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

    public Command driveDoubleConeCommand(Supplier<ChassisSpeeds> speeds, Supplier<Translation2d> centerOfRot) {
        return new RunCommand(() -> driveDoubleConePercent(speeds.get().vxMetersPerSecond,
                speeds.get().vyMetersPerSecond, speeds.get().omegaRadiansPerSecond, true, centerOfRot.get()),
                new Subsystem[] {})
                .andThen(new InstantCommand(
                        () -> driveDesaturatedCommand(() -> new ChassisSpeeds(), () -> new Translation2d())));
    }

    public Command driveDesaturatedCommand(Supplier<ChassisSpeeds> speeds, Supplier<Translation2d> centerOfRot) {
        return new RunCommand(() -> driveDesaturated(speeds.get().vxMetersPerSecond,
                speeds.get().vyMetersPerSecond, speeds.get().omegaRadiansPerSecond, true, centerOfRot.get()),
                new Subsystem[] {})
                .andThen(new InstantCommand(
                        () -> driveDesaturatedCommand(() -> new ChassisSpeeds(), () -> new Translation2d())));
    }

    public Command resetOdometryAprilTag() {
        return new InstantCommand(() -> resetOdometry(getAprilTagPose()));

    }

    public Pose2d getAprilTagPose() {
        for (AprilTagVision vision : visions) {
            if (vision.isTarget()) {
                return vision.getAprilTagPose2d();
            }
        }
        return getPose();
    }

    public boolean getRotAccuracy() {
        return Math.toDegrees(Math.abs(SwerveAlgorithms.angleDistance(
                DriveWeightCommand.getAngle(), getPose().getRotation().getRadians()))) < 3;
    }
}