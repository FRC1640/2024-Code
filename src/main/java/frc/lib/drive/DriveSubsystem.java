package frc.lib.drive;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

// for pose est.
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
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

public class DriveSubsystem extends SubsystemBase {

    Gyro gyro;
    ArrayList<AprilTagVision> visions = new ArrayList<AprilTagVision>();

    private Module frontLeft;
    private Module frontRight;
    private Module backLeft;
    private Module backRight;

    private final double maxSpeed = Constants.SwerveDriveDimensions.maxSpeed;

    private SwerveModuleState[] desiredSwerveStates = new SwerveModuleState[] {};

    // SwerveDriveOdometry odometry;
    SwerveDrivePoseEstimator swervePoseEstimator;
    SysIdRoutine sysIdRoutine;
    Pose2d odometryPose = new Pose2d();

    private Command testCommand;

    public DriveSubsystem(Gyro gyro, AprilTagVision vision) {
        this.gyro = gyro;
        visions.add(vision);
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
                VecBuilder.fill(0.05, 0.05, 0.05),
                VecBuilder.fill(AprilTagVisionConstants.xyStdDev, AprilTagVisionConstants.xyStdDev, AprilTagVisionConstants.thetaStdDev));

        // Configure pathplanner
        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetOdometry,
                () -> SwerveDriveDimensions.kinematics.toChassisSpeeds(getActualSwerveStates()),
                this::driveChassisSpeedsDesaturated,
                new HolonomicPathFollowerConfig(
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
            if (vision.isTarget() && vision.isPoseValid(vision.getAprilTagPose2d())) {

                double distConst = Math.pow(vision.getDistance(), 2.0); // distance standard deviation constant

                double velConst = Math.pow(Math.hypot(SwerveDriveDimensions.kinematics.toChassisSpeeds(
                        getActualSwerveStates()).vxMetersPerSecond,
                        SwerveDriveDimensions.kinematics.toChassisSpeeds(getActualSwerveStates()).vyMetersPerSecond),
                        1);
                swervePoseEstimator.addVisionMeasurement(vision.getAprilTagPose2d(), vision.getLatency(),
                        VecBuilder.fill(AprilTagVisionConstants.xyStdDev * distConst + velConst / 5,
                                AprilTagVisionConstants.xyStdDev * distConst + velConst / 5,
                                AprilTagVisionConstants.thetaStdDev * distConst + velConst / 5));
            }

        }
        // update odometry
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
        frontLeft.setDesiredStateMetersPerSecond(swerveModuleStates[0]);
        frontRight.setDesiredStateMetersPerSecond(swerveModuleStates[1]);
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
    public Pose2d getAprilTagPose(){
        for (AprilTagVision vision : visions) {
            if (vision.isTarget()){
                return vision.getAprilTagPose2d();
            }
        }
        return getPose();
    }

    // motor test methods

    private void testFrontLeftAngle(double angle) {
        frontLeft.setDesiredStateMetersPerSecond(new SwerveModuleState(frontLeft.getVelocity(), new Rotation2d(angle)));
    }

    private void testFrontRightAngle(double angle) {
        frontRight.setDesiredStateMetersPerSecond(new SwerveModuleState(frontRight.getVelocity(), new Rotation2d(angle)));
    }

    private void testBackLeftAngle(double angle) {
        backLeft.setDesiredStateMetersPerSecond(new SwerveModuleState(backLeft.getVelocity(), new Rotation2d(angle)));
    }

    private void testBackRightAngle(double angle) {
        backRight.setDesiredStateMetersPerSecond(new SwerveModuleState(backRight.getVelocity(), new Rotation2d(angle)));
    }

    private void testFrontLeftVelocity(double velocity) {
        frontLeft.setDesiredStateMetersPerSecond(new SwerveModuleState(velocity, new Rotation2d(frontLeft.getState().angle.getDegrees())));
    }

    private void testFrontRightVelocity(double velocity) {
        frontRight.setDesiredStateMetersPerSecond(new SwerveModuleState(velocity, new Rotation2d(frontRight.getState().angle.getDegrees())));
    }

    private void testBackLeftVelocity(double velocity) {
        backLeft.setDesiredStateMetersPerSecond(new SwerveModuleState(velocity, new Rotation2d(backLeft.getState().angle.getDegrees())));
    }

    private void testBackRightVelocity(double velocity) {
        backRight.setDesiredStateMetersPerSecond(new SwerveModuleState(velocity, new Rotation2d(backRight.getState().angle.getDegrees())));
    }

    public Command testFrontLeftAngleCommand(DoubleSupplier angle) {
        Command c = new Command() {
            @Override
            public void initialize() {
                testCommand = this;
            }
            @Override
            public void execute() {
                testFrontLeftAngle(angle.getAsDouble());
            }
            @Override
            public boolean isFinished() {
                return testCommand != this;
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command testFrontRightAngleCommand(DoubleSupplier angle) {
        Command c = new Command() {
            @Override
            public void initialize() {
                testCommand = this;
            }
            @Override
            public void execute() {
                testFrontRightAngle(angle.getAsDouble());
            }
            @Override
            public boolean isFinished() {
                return testCommand != this;
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command testBackLeftAngleCommand(DoubleSupplier angle) {
        Command c = new Command() {
            @Override
            public void initialize() {
                testCommand = this;
            }
            @Override
            public void execute() {
                testBackLeftAngle(angle.getAsDouble());
            }
            @Override
            public boolean isFinished() {
                return testCommand != this;
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command testBackRightAngleCommand(DoubleSupplier angle) {
        Command c = new Command() {
            @Override
            public void initialize() {
                testCommand = this;
            }
            @Override
            public void execute() {
                testBackRightAngle(angle.getAsDouble());
            }
            @Override
            public boolean isFinished() {
                return testCommand != this;
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command testFrontLeftVelocityCommand(DoubleSupplier velocity) {
        Command c = new Command() {
            @Override
            public void initialize() {
                testCommand = this;
            }
            @Override
            public void execute() {
                testFrontLeftVelocity(velocity.getAsDouble());
            }
            @Override
            public boolean isFinished() {
                return testCommand != this;
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command testFrontRightVelocityCommand(DoubleSupplier velocity) {
        Command c = new Command() {
            @Override
            public void initialize() {
                testCommand = this;
            }
            @Override
            public void execute() {
                testFrontRightVelocity(velocity.getAsDouble());
            }
            @Override
            public boolean isFinished() {
                return testCommand != this;
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command testBackLeftVelocityCommand(DoubleSupplier velocity) {
        Command c = new Command() {
            @Override
            public void initialize() {
                testCommand = this;
            }
            @Override
            public void execute() {
                testBackLeftVelocity(velocity.getAsDouble());
            }
            @Override
            public boolean isFinished() {
                return testCommand != this;
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command testBackRightVelocityCommand(DoubleSupplier velocity) {
        Command c = new Command() {
            @Override
            public void initialize() {
                testCommand = this;
            }
            @Override
            public void execute() {
                testBackRightVelocity(velocity.getAsDouble());
            }
            @Override
            public boolean isFinished() {
                return testCommand != this;
            }
        };
        c.addRequirements(this);
        return c;
    }

    public double getFrontLeftAngle() {
        return frontLeft.getState().angle.getDegrees();
    }

    public double getFrontRightAngle() {
        return frontRight.getState().angle.getDegrees();
    }

    public double getBackLeftAngle() {
        return backLeft.getState().angle.getDegrees();
    }

    public double getBackRightAngle() {
        return backRight.getState().angle.getDegrees();
    }

    public double getFrontLeftVelocity() {
        return frontLeft.getVelocity();
    }

    public double getFrontRightVelocity() {
        return frontRight.getVelocity();
    }

    public double getBackLeftVelocity() {
        return backLeft.getVelocity();
    }

    public double getBackRightVelocity() {
        return backRight.getVelocity();
    }
}
