package frc.robot.subsystems.drive.DriveWeights;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveForwardWeight implements DriveWeight {
    double time;
    double velocity;
    long initTime = -1;
    public DriveForwardWeight(double time, double velocity){
        this.time = time;
        this.velocity = velocity;
    }

    @Override
    public ChassisSpeeds getSpeeds(){
        if (initTime == -1){
            initTime = System.currentTimeMillis();
        }
        if (initTime!= -1 && initTime + time > System.currentTimeMillis()){
            return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(velocity, 0,0), new Rotation2d(0));
        }
        else{
            return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(0, 0,0), new Rotation2d(0));
            
        }
    }
}
