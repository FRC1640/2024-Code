package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Robot.Mode;
import frc.robot.sensors.Gyro.Gyro;
import frc.robot.sensors.Gyro.GyroIO;
import frc.robot.sensors.Gyro.GyroIONavX;
import frc.robot.subsystems.drive.PivotConfig.PivotId;

public class DriveSubsystem extends SubsystemBase{
    Gyro gyro;
    public static final double y = Constants.SwerveDriveDimensions.wheelYPos; // 10.375"
    public static final double x = Constants.SwerveDriveDimensions.wheelXPos; // 12.375"

    private final Translation2d frontLeftLocation = new Translation2d(x, y);
    private final Translation2d frontRightLocation = new Translation2d(x, -y);
    private final Translation2d backLeftLocation = new Translation2d(-x, y);
    private final Translation2d backRightLocation = new Translation2d(-x, -y);

    private Module frontLeft;
    private Module frontRight;
    private Module backLeft;
    private Module backRight;

    private final double maxSpeed = Constants.SwerveDriveDimensions.maxSpeed;


      private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    public DriveSubsystem(Gyro gyro){
        this.gyro = gyro;
        switch (Robot.getMode()) {
            case REAL:
                frontLeft = new Module(new ModuleIOSparkMax(PivotConfig.getConfig(PivotId.FL)), PivotId.FL);
                frontRight = new Module(new ModuleIOSparkMax(PivotConfig.getConfig(PivotId.FR)), PivotId.FR);
                backLeft = new Module(new ModuleIOSparkMax(PivotConfig.getConfig(PivotId.BL)), PivotId.BL);
                backRight = new Module(new ModuleIOSparkMax(PivotConfig.getConfig(PivotId.BR)), PivotId.BR);
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
    }

    @Override
    public void periodic(){
        frontLeft.periodic();
        frontRight.periodic();
        backLeft.periodic();
        backRight.periodic();
        Logger.recordOutput("Drive/SwerveStates", new SwerveModuleState[] {frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()});
    }
    /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drivePercent(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    
    xSpeed = xSpeed * maxSpeed;
    ySpeed = ySpeed * maxSpeed;
    rot = rot * maxSpeed;

    var swerveModuleStates = kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
                new Rotation2d(gyro.getAngleRotation2d().getRadians()))
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);

    Logger.recordOutput("Drive/DesiredSwerveStates", swerveModuleStates);
  }
}
