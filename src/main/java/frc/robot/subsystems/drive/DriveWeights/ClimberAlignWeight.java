package frc.robot.subsystems.drive.DriveWeights;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ClimberAlignWeight implements DriveWeight {
    BooleanSupplier sensor1;
    BooleanSupplier sensor2;
    public ClimberAlignWeight(BooleanSupplier sensor1, BooleanSupplier sensor2){
        this.sensor1 = sensor1;
        this.sensor2 = sensor2;
    }

    @Override
    public ChassisSpeeds getSpeeds(){
        if (!sensor1.getAsBoolean() && !sensor1.getAsBoolean()){
            return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(-0.01, 0,0), new Rotation2d(0));
        } else if (!sensor1.getAsBoolean() && sensor1.getAsBoolean()){
            return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(0, 0, 0.05), new Rotation2d(0));
        } else if (sensor1.getAsBoolean() && !sensor1.getAsBoolean()){
            return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(0, 0, -0.05), new Rotation2d(0));
        } else {
            return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(0, 0,0), new Rotation2d(0));
        }
    }
    @Override
    public boolean cancelCondition() {
        return (sensor1.getAsBoolean() && sensor1.getAsBoolean());
    }
}
