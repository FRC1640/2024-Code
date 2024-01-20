package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

// for pose est.
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.pathplanning.LocalADStarAK;
import frc.lib.swerve.SwerveAlgorithms;
import frc.lib.sysid.SwerveDriveSysidRoutine;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.PivotId;
import frc.robot.Constants.SwerveDriveDimensions;
import frc.robot.Constants.VisionConstants;
import frc.robot.sensors.Gyro.Gyro;
import frc.robot.sensors.Vision.AprilTagVision;
import frc.robot.subsystems.drive.Module.ModuleIO;
import frc.robot.subsystems.drive.Module.ModuleIOSim;
import frc.robot.subsystems.drive.Module.ModuleIOSparkMax;
import frc.robot.subsystems.drive.Module.Module;

public class DriveSubsystem extends SubsystemBase {
    Gyro gyro;
    AprilTagVision vision;

    private Module frontLeft;
    private Module frontRight;
    private Module backLeft;
    private Module backRight;

    private final double maxSpeed = Constants.SwerveDriveDimensions.maxSpeed;

    private SwerveModuleState[] desiredSwerveStates = new SwerveModuleState[] {};

    //SwerveDriveOdometry odometry;
    SwerveDrivePoseEstimator swervePoseEstimator; // swerve pose estimator is an alt. for swerve odometry
    SysIdRoutine sysIdRoutine;
    Pose2d odometryPose = new Pose2d();
    public DriveSubsystem(Gyro gyro, AprilTagVision vision){
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


       //Create Pose Estimator
        //odometry = new SwerveDriveOdometry(SwerveDriveDimensions.kinematics, gyro.getAngleRotation2d(), getModulePositionsArray());
        swervePoseEstimator = new SwerveDrivePoseEstimator(
            SwerveDriveDimensions.kinematics, 
            gyro.getAngleRotation2d(), 
            getModulePositionsArray(), 
            new Pose2d(),
                VecBuilder.fill(0.05, 0.05, 0.05),
                VecBuilder.fill(VisionConstants.xyStdDev, VisionConstants.xyStdDev, VisionConstants.thetaStdDev));// THIS IS SUPPOSED TO BE THE starting standard deviations

        //Configure pathplanner
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
    }
    public SwerveModuleState[] getActualSwerveStates() {
        return new SwerveModuleState[] { frontLeft.getState(), frontRight.getState(), backLeft.getState(),
                backRight.getState() };
    }

    public SwerveModuleState[] getDesiredSwerveStates() {
        return desiredSwerveStates;
    }


  public void updateOdometry(){
    if (vision.isTarget()){

        double distConst = Math.pow(vision.getDistance(), 2.0);
        swervePoseEstimator.addVisionMeasurement(vision.getAprilTagPose2d(), vision.getLatency(), 
        VecBuilder.fill(VisionConstants.xyStdDev * distConst, 
        VisionConstants.xyStdDev * distConst, VisionConstants.thetaStdDev * distConst));
    }
    odometryPose = swervePoseEstimator.update(gyro.getRawAngleRotation2d(), getModulePositionsArray());
    
  }

  public void resetOdometry(Pose2d newPose){
    swervePoseEstimator.resetPosition(gyro.getRawAngleRotation2d(), getModulePositionsArray(), newPose);
    odometryPose = newPose;
  }

  public Pose2d getPose(){
    return odometryPose;
  }

  public SwerveModulePosition[] getModulePositionsArray(){
    return new SwerveModulePosition[] {
        frontLeft.getPosition(), 
        frontRight.getPosition(), 
        backLeft.getPosition(), 
        backRight.getPosition()};
  }

  public Command resetOdometryCommand(Pose2d newPose){
    return new InstantCommand(() -> resetOdometry(newPose));
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
    public void drivePercentDesaturated(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

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
    public void drivePercentDoubleCone(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
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

    public void drivePercentDoubleCone(double xSpeed, double ySpeed, double rot, boolean fieldRelative,
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

    public void driveChassisSpeedsNoScaling(ChassisSpeeds speeds) {
        SwerveModuleState[] swerveModuleStates = SwerveAlgorithms.rawSpeeds(speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
        desiredSwerveStates = swerveModuleStates;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

}
