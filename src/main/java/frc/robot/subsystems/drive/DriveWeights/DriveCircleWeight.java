package frc.robot.subsystems.drive.DriveWeights;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveCircleWeight implements DriveWeight {
    double time;
    double velocity;
    long initTime = -1;
    boolean timeUp = false;
    public DriveCircleWeight(double time, double velocity){
        this.time = time*1000;
        this.velocity = velocity;
        initTime = -1;
        timeUp = false;
    }

    @Override
    public ChassisSpeeds getSpeeds(){
        if (initTime == -1){
            initTime = System.currentTimeMillis();
        }
        if (initTime != -1){
            double timespent = (initTime + time - System.currentTimeMillis());
            return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(velocity*Math.cos(timespent*2*Math.PI/time), velocity*Math.sin(timespent*2*Math.PI/time),0), new Rotation2d(0));
        }
        else{
            return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(0, 0,0), new Rotation2d(0));
        }
    }

    public void reset() {
        initTime = -1;
        timeUp = false;
    }
}
