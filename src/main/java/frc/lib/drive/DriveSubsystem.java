package frc.lib.drive;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.drive.Module.Module;
import frc.lib.drive.Module.ModuleIO;
import frc.lib.drive.Module.ModuleIOSim;
import frc.lib.drive.Module.ModuleIOSparkMax;
import frc.lib.sysid.SwerveDriveSysidRoutine;
import frc.lib.vision.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.DashboardInit;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.PivotId;
import frc.robot.Constants.SwerveDriveDimensions;
import frc.robot.Robot;
import frc.robot.sensors.Gyro.Gyro;
import frc.robot.sensors.Vision.AprilTagVision.AprilTagVision;
import frc.robot.sensors.Vision.MLVision.MLVision;
import frc.robot.util.pathplanning.LocalADStarAK;

public class DriveSubsystem extends SubsystemBase {

    Gyro gyro;
    ArrayList<AprilTagVision> visions = new ArrayList<AprilTagVision>();

    private Module frontLeft;
    private Module frontRight;
    private Module backLeft;
    private Module backRight;
    boolean usedAprilTag;
    boolean aprilTagInAuto = false;

    static final Lock odometryLock = new ReentrantLock();

    PIDController pidr = frc.robot.Constants.PIDConstants.constructPID(frc.robot.Constants.PIDConstants.rotPID,
            "RotateToAngle1");

    private final double maxSpeed = Constants.SwerveDriveDimensions.maxSpeed;

    private SwerveModuleState[] desiredSwerveStates = new SwerveModuleState[] {};

    double dynamicThreshold = 0.8;

    // SwerveDriveOdometry odometry;
    SwerveDrivePoseEstimator swervePoseEstimator;
    SysIdRoutine sysIdRoutine;
    Pose2d odometryPose = new Pose2d();

    private Rotation2d rawGyroRotation = new Rotation2d();

    private SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            };

    

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
                rawGyroRotation,
                getModulePositionsArray(),
                new Pose2d(),
                VecBuilder.fill(0.1, 0.1, 0.00001),
                VecBuilder.fill(2, 2, 9999999));

        // Configure pathplanner
        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetOdometryAuton,
                () -> SwerveDriveDimensions.kinematics.toChassisSpeeds(getActualSwerveStates()),
                this::driveChassisSpeedsDesaturated,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(2.5, 0, 0),
                        new PIDConstants(2.5, 0, 0),
                        4,
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

        SparkMaxOdometryThread.getInstance().start();
    }

    @Override
    public void periodic() {
        // Update modules
        odometryLock.lock();
        frontLeft.periodic();
        frontRight.periodic();
        backLeft.periodic();
        backRight.periodic();
        gyro.periodic();
        odometryLock.unlock();
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
        // update odometry, thanks to:
        // https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive

        double[] sampleTimestamps = frontLeft.getOdometryTimestamps();
        int sampleCount = sampleTimestamps.length;

        Module[] modules = new Module[] { frontLeft, frontRight, backLeft, backRight };

        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters
                                - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }
            // Update gyro angle
            if (gyro.isTrustworthy()) {
                // Use the real gyro angle
                rawGyroRotation = gyro.getOdometryPositions()[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = SwerveDriveDimensions.kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            odometryPose = swervePoseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }
        for (AprilTagVision vision : visions) {
            Pose2d pose = new Pose2d();
            if (Robot.isDisabled){
                pose = vision.getAprilTagPose2dRot();
            }
            else{
                pose = vision.getAprilTagPose2d();
            }
            // pose = vision.getAprilTagPose2dRot();
            LimelightHelpers.SetRobotOrientation("limelight" + vision.getName(),
                    getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
            if (vision.isPoseValid(pose)
                    && (Robot.inTeleop || aprilTagInAuto || Robot.isDisabled)
                    && vision.getNumVisibleTags() != 0 && Math.abs(gyro.getAngularVelDegreesPerSecond()) < 100
                    && vision.getDistance() < 7.3
                    // && !(Robot.isDisabled && vision.getName() == "-back")
                    ) {
                double distanceToTag = vision.getDistance();
               
                double distConst = 1 + (distanceToTag * distanceToTag);
                double poseDifference = pose.getTranslation()
                        .getDistance(getPose().getTranslation());

                double posDifX = pose.getTranslation().getX() - getPose().getX();
                double posDifY = pose.getTranslation().getY() - getPose().getY();

                double posDifTheta = Math.toDegrees(SwerveAlgorithms.angleDistance(
                    vision.getAprilTagPose2dRot().getRotation().getRadians(), getPose().getRotation().getRadians()));

                double speed = Math.hypot(SwerveDriveDimensions.kinematics.toChassisSpeeds(
                        getActualSwerveStates()).vxMetersPerSecond,
                        SwerveDriveDimensions.kinematics
                                .toChassisSpeeds(getActualSwerveStates()).vyMetersPerSecond);

                boolean mt1 = false;
                double xy = 0.65;
                if (speed > 1.5){
                    xy = 0.9;
                }
                double theta = Double.MAX_VALUE;
                 if ((vision.getDistance() < 3.75 && speed < 2.5 && vision.getNumVisibleTags() > 1)){
                    pose = vision.getAprilTagPose2dRot();
                    mt1 = true;
                    xy = 1.1;
                 }

                // double xy = AprilTagVisionConstants.xyStdDev;
                
                // if (!Robot.inTeleop){
                // xy = 2.5;
                // if (poseDifference > 0.3){
                // xy = 0.5;
                // }
                // }
                // if ((speed > 1 && vision.getDistance() > 3.5)
                //         || ((vision.getDistance() > 5 || speed > 1) && vision.getNumVisibleTags() == 1)) {
                //     xy = 0.9;
                // }
                // if (vision.getNumVisibleTags() > 2){
                // xy = 0.25;
                // }
                
                
                boolean useEstimate = true;

                // if ((vision.getDistance() > 4 && vision.getNumVisibleTags() >= 2 &&
                // vision.getName() == "-back")
                // || (vision.getDistance() > 3 && vision.getNumVisibleTags() == 1)
                // || (vision.getDistance() > 6)) {
                // useEstimate = false;
                // }
                // if (vision.getNumVisibleTags() >= 2 &&
                // Arrays.stream(vision.getDistances()).max().getAsDouble() < 4) {
                // xy = 0.25;
                // } else {
                // if (speed < 1 && vision.getDistance() < 2.75) {
                // xy = 0.5;
                // } else if (speed < 0.5 && vision.getDistance() < 4) {
                // xy = 1.5;
                // }
                // }

                for (int i = 0; i < vision.getTagPoses().length; i++) {
                    Logger.recordOutput(vision.getName() + "/PosDifference" + i, poseDifference);
                    Logger.recordOutput(vision.getName() + "/PosDifX" + i, posDifX);
                    Logger.recordOutput(vision.getName() + "/PosDifY" + i, posDifY);
                    Logger.recordOutput(vision.getName() + "/DynamicThreshold" + i, dynamicThreshold);
                    Logger.recordOutput(vision.getName() + "/DynamicThreshold" + i, dynamicThreshold);
                }
                if (useEstimate) {
                    swervePoseEstimator.addVisionMeasurement(pose, vision.getLatency(),
                            VecBuilder.fill(xy,
                                    xy,
                                    Robot.isDisabled || mt1?0.00001:Double.MAX_VALUE));

                }
            }
        }
    }

    private void resetOdometry(Pose2d newPose) {
        swervePoseEstimator.resetPosition(gyro.getRawAngleRotation2d(), getModulePositionsArray(), newPose);
        odometryPose = newPose;
    }

    public void setBrakeMode(boolean brake) {
        frontLeft.setBrakeMode(brake);
        frontRight.setBrakeMode(brake);
        backLeft.setBrakeMode(brake);
        backRight.setBrakeMode(brake);
    }

    public Command pidPivotsCommand() {
        return new RunCommand(() -> {
            frontLeft.pidModule(0);
            frontRight.pidModule(0);
            backLeft.pidModule(0);
            backRight.pidModule(0);
        }, this).until(() -> ((frontLeft.getError())
                && (frontRight.getError())
                && (backLeft.getError())
                && (backRight.getError())));
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
        if (pose.getTranslation().getDistance(getPose().getTranslation()) > -1) {
            resetOdometry(pose);
        }
        // gyro.setOffset(gyro.getRawAngleRadians() - pose.getRotation().getRadians());

    }

    public void resetPivots() {
        frontLeft.resetSteer();
        frontRight.resetSteer();
        backLeft.resetSteer();
        backRight.resetSteer();
    }

    public Translation2d getFieldCentricVelocity(){
        ChassisSpeeds s = Constants.SwerveDriveDimensions.kinematics.toChassisSpeeds(getActualSwerveStates());
        Translation2d unRotated = new Translation2d(s.vxMetersPerSecond, s.vyMetersPerSecond);
        return unRotated.rotateBy(getPose().getRotation());
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
                driveDoubleConePercent(0, 0, 0, false, new Translation2d(), false);
            }

            @Override
            public void execute() {
                angle = angleSupplier.getAsDouble();
                System.out.println(Math.toDegrees(angle));
                
                Logger.recordOutput("RotCommand", true);

                double o;
                o = pidr.calculate(-SwerveAlgorithms.angleDistance(odometryPose.getRotation().getRadians(),
                        angle), 0);
                if (Math.abs(o) < 0.005) {
                    o = 0;
                }
                o = MathUtil.clamp(o, -1, 1);

                // Logger.recordOutput("AutoRotateEnded", (Math.abs(SwerveAlgorithms.angleDistance(odometryPose.getRotation().getRadians(),
                //         angle)) < Math.toRadians(0.5)));

                driveDoubleConePercent(0, 0, o, false, new Translation2d(), false);

            }

            @Override
            public boolean isFinished() {
                return (Math.abs(SwerveAlgorithms.angleDistance(odometryPose.getRotation().getRadians(),
                        angle)) < Math.toRadians(1));
                // return false;
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

    public void setAprilTagInAuto(boolean value) {
        aprilTagInAuto = value;
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

    public void driveDoubleCone(double xSpeed, double ySpeed, double rot, boolean fieldRelative,
            Translation2d centerOfRotation) {
        rot = rot
                / SwerveAlgorithms.computeMaxNorm(SwerveDriveDimensions.positions, centerOfRotation);
        SwerveModuleState[] swerveModuleStates = SwerveAlgorithms.doubleCone(xSpeed, ySpeed, rot,
                gyro.getAngleRotation2d().getRadians(), fieldRelative, centerOfRotation, false);
        frontLeft.setDesiredStateMetersPerSecond(swerveModuleStates[0]);
        frontRight.setDesiredStateMetersPerSecond(swerveModuleStates[1]);
        backLeft.setDesiredStateMetersPerSecond(swerveModuleStates[2]);
        backRight.setDesiredStateMetersPerSecond(swerveModuleStates[3]);
        desiredSwerveStates = swerveModuleStates;
    }

    public void driveDoubleConePercent(double xSpeed, double ySpeed, double rot, boolean fieldRelative,
            Translation2d centerOfRotation, boolean lockRotation) {
        xSpeed = xSpeed * maxSpeed;
        ySpeed = ySpeed * maxSpeed;
        rot = rot * maxSpeed
                / SwerveAlgorithms.computeMaxNorm(SwerveDriveDimensions.positions, centerOfRotation);
        SwerveModuleState[] swerveModuleStates = SwerveAlgorithms.doubleCone(xSpeed, ySpeed, rot,
                gyro.getAngleRotation2d().getRadians(), fieldRelative, centerOfRotation, lockRotation);

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

    public Command driveDoubleConeCommand(Supplier<ChassisSpeeds> speeds, Supplier<Translation2d> centerOfRot,
            BooleanSupplier lockRotation) {
        return new RunCommand(() -> driveDoubleConePercent(speeds.get().vxMetersPerSecond,
                speeds.get().vyMetersPerSecond, speeds.get().omegaRadiansPerSecond, true, centerOfRot.get(),
                lockRotation.getAsBoolean()),
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
                return vision.getAprilTagPose2dRot();
            }
        }
        return getPose();
    }

    public Command pathWithMovableEndpoint(String pathName, Supplier<Translation2d> endpoint){
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        path.preventFlipping = true;
        path.getAllPathPoints().set(path.getAllPathPoints().size() - 1, new PathPoint(endpoint.get()));
        return AutoBuilder.followPath(path);
    }


    public Command autoGeneratedPath(Supplier<Translation2d> endState, BooleanSupplier condition){
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
        new Pose2d(getPose().getTranslation(), getFieldCentricVelocity().getNorm() < 0.01 ? getFieldCentricVelocity().getAngle() : getPose().getRotation()),
        new Pose2d(endState.get(), getPose().getRotation())
        );

        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            new PathConstraints(1.5, 1, 2 * Math.PI, 4 * Math.PI),
            new GoalEndState(0, getPose().getRotation()));

        path.preventFlipping = true;
        return new FollowPathHolonomic(path,this::getPose,
                () -> SwerveDriveDimensions.kinematics.toChassisSpeeds(getActualSwerveStates()),
                this::driveChassisSpeedsDesaturated,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(2.5, 0, 0),
                        new PIDConstants(2.5, 0, 0),
                        4,
                        SwerveAlgorithms.computeMaxNorm(SwerveDriveDimensions.positions, new Translation2d(0, 0)),
                        new ReplanningConfig()),
                () -> DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red,
                this).until(condition);
    }

    public void setRotationTargetAuto(Supplier<Rotation2d> rotation){
        PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.of(rotation.get()));
    }
    public void setRotationTargetAuto(){
        PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.empty());
    }

    public boolean getRotAccuracy() {
        return Math.toDegrees(Math.abs(SwerveAlgorithms.angleDistance(
                DriveWeightCommand.getAngle(), getPose().getRotation().getRadians()))) < 3;
    }

    public void rotatePivots(DoubleSupplier angle) {
        frontLeft.setDesiredStateMetersPerSecond(new SwerveModuleState(0, Rotation2d.fromDegrees(angle.getAsDouble())));
        frontRight
                .setDesiredStateMetersPerSecond(new SwerveModuleState(0, Rotation2d.fromDegrees(angle.getAsDouble())));
        backLeft.setDesiredStateMetersPerSecond(new SwerveModuleState(0, Rotation2d.fromDegrees(angle.getAsDouble())));
        frontRight
                .setDesiredStateMetersPerSecond(new SwerveModuleState(0, Rotation2d.fromDegrees(angle.getAsDouble())));
    }
}