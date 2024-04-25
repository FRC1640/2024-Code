package frc.robot.subsystems.drive.DriveWeights;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.drive.DriveWeight;

public class DriveForwardRobotRelativeWeight implements DriveWeight {
    double time;
    double velocity;
    long initTime = -1;
    boolean timeUp = false;
    Supplier<Rotation2d> angleSupplier;
    public DriveForwardRobotRelativeWeight(double time, double velocity, Supplier<Rotation2d> angleSupplier){
        this.time = time;
        this.velocity = velocity;
        this.angleSupplier = angleSupplier;
        initTime = -1;
        timeUp = false;

    }

    @Override
    public ChassisSpeeds getSpeeds(){
        if (initTime == -1){
            initTime = System.currentTimeMillis();
        }
        if (initTime != -1 && initTime + time > System.currentTimeMillis()){
            return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(velocity, 0,0), angleSupplier.get());
        }
        else{
            timeUp = true;
            return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(0, 0,0), angleSupplier.get());
        }
    }
    @Override
    public boolean cancelCondition() {
        if (timeUp == true){
            initTime = -1;
        }
        return timeUp;
    }
}

