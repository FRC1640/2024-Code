package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.swerve.SwerveAlgorithms;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.PivotId;
import frc.robot.sensors.Gyro.Gyro;

public class DriveSubsystem extends SubsystemBase{
    Gyro gyro;

    private Module frontLeft;
    private Module frontRight;
    private Module backLeft;
    private Module backRight;

    private final double maxSpeed = Constants.SwerveDriveDimensions.maxSpeed;

    private SwerveModuleState[] desiredSwerveStates = new SwerveModuleState[]{};

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
    }

    @Override
    public void periodic(){
        frontLeft.periodic();
        frontRight.periodic();
        backLeft.periodic();
        backRight.periodic();
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
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drivePercentDesaturated(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    
    xSpeed = xSpeed * maxSpeed;
    ySpeed = ySpeed * maxSpeed;
    rot = rot * maxSpeed;

    SwerveModuleState[] swerveModuleStates = SwerveAlgorithms.desaturated(xSpeed, ySpeed, rot, 
        gyro.getAngleRotation2d().getRadians(), fieldRelative);

    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
    desiredSwerveStates = swerveModuleStates;    
  }
  public void drivePercentDoubleCone(double xSpeed, double ySpeed, double rot, boolean fieldRelative){
    xSpeed = xSpeed * maxSpeed;
    ySpeed = ySpeed * maxSpeed;
    rot = rot * maxSpeed;
    SwerveModuleState[] swerveModuleStates = SwerveAlgorithms.doubleCone(xSpeed, ySpeed, rot, 
        gyro.getAngleRotation2d().getRadians(), fieldRelative);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
    desiredSwerveStates = swerveModuleStates;
  }
}
