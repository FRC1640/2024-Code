package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.swerve.SwerveAlgorithms;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.PivotId;
import frc.robot.Constants.SwerveDriveDimensions;
import frc.robot.sensors.Gyro.Gyro;

public class DriveSubsystem extends SubsystemBase{
    Gyro gyro;

    private Module frontLeft;
    private Module frontRight;
    private Module backLeft;
    private Module backRight;

    private final double maxSpeed = Constants.SwerveDriveDimensions.maxSpeed;

    private SwerveModuleState[] desiredSwerveStates = new SwerveModuleState[]{};

    SwerveDriveOdometry odometry;

    Pose2d odometryPose = new Pose2d();

    double angularVelDegreesPerSecond;

    Rotation2d angleOffset = new Rotation2d(0);

    public DriveSubsystem(Gyro gyro){
        this.gyro = gyro;
        
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
                frontLeft = new Module(new ModuleIO() {}, PivotId.FL);
                frontRight = new Module(new ModuleIO() {}, PivotId.FR);
                backLeft = new Module(new ModuleIO() {}, PivotId.BL);
                backRight = new Module(new ModuleIO() {}, PivotId.BR);
                break;
        }  


        //Create odometry
        odometry = new SwerveDriveOdometry(SwerveDriveDimensions.kinematics, gyro.getAngleRotation2d(), getModulePositionsArray());
    }

    @Override
    public void periodic(){
        //Update modules
        frontLeft.periodic();
        frontRight.periodic();
        backLeft.periodic();
        backRight.periodic();
        //Update odometry
        updateOdometry();
        //Log odometry
        Logger.recordOutput("Drive/OdometryPosition", odometryPose);
        //Log swerve states
        Logger.recordOutput("Drive/SwerveStates", getActualSwerveStates());
        Logger.recordOutput("Drive/DesiredSwerveStates", getDesiredSwerveStates());
    }

    public SwerveModuleState[] getActualSwerveStates(){
        return new SwerveModuleState[] {frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()};
    }
    public SwerveModuleState[] getDesiredSwerveStates(){
        return desiredSwerveStates;
    }
   /**
   * Method to drive the robot using joystick info and desaturated swerve algorithm.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drivePercentDesaturated(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    
    xSpeed = xSpeed * maxSpeed;
    ySpeed = ySpeed * maxSpeed;
    rot = rot * maxSpeed / SwerveAlgorithms.computeMaxNorm(SwerveDriveDimensions.positions, new Translation2d(0,0));

    SwerveModuleState[] swerveModuleStates = SwerveAlgorithms.desaturated(xSpeed, ySpeed, rot, 
        gyro.getAngleRotation2d().getRadians(), fieldRelative);

    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
    desiredSwerveStates = swerveModuleStates;
    angularVelDegreesPerSecond = Math.toDegrees(rot);
  }
   /**
   * Method to drive the robot using joystick info and double cone swerve algorithm.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drivePercentDoubleCone(double xSpeed, double ySpeed, double rot, boolean fieldRelative){
    xSpeed = xSpeed * maxSpeed;
    ySpeed = ySpeed * maxSpeed;
    rot = rot * maxSpeed / SwerveAlgorithms.computeMaxNorm(SwerveDriveDimensions.positions, new Translation2d(0,0));
    SwerveModuleState[] swerveModuleStates = SwerveAlgorithms.doubleCone(xSpeed, ySpeed, rot, 
        gyro.getAngleRotation2d().getRadians(), fieldRelative);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
    desiredSwerveStates = swerveModuleStates;
    angularVelDegreesPerSecond = Math.toDegrees(rot);
  }

  public void updateOdometry(){
    odometryPose = odometry.update(gyro.getAngleRotation2d().plus(angleOffset), getModulePositionsArray());
  }

  public void resetOdometry(Pose2d newPose){

    odometry.resetPosition(gyro.getAngleRotation2d(), getModulePositionsArray(), newPose);
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

  public double getAngularVelDegreesPerSecond(){
    return angularVelDegreesPerSecond;
  }

  public void setAngleOffset(Rotation2d offset){
    angleOffset = offset;
  }
}
