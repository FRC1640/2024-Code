package frc.robot.subsystems.drive.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.sensors.Gyro.Gyro;
import frc.robot.subsystems.drive.DriveSubsystem;

public class ResetGyro extends Command{

    Gyro gyro;
    DriveSubsystem driveSubsystem;
    public ResetGyro(DriveSubsystem driveSubsystem,Gyro gyro){
        this.gyro = gyro;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = driveSubsystem.getPose();
        Rotation2d oldAngle = gyro.getAngleRotation2d();
        gyro.reset();
        driveSubsystem.resetOdometry(currentPose);
        driveSubsystem.setAngleOffset(oldAngle);
    }
    
    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
