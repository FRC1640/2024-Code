package frc.robot.subsystems.drive.DriveWeights;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ClimberAlignWeight implements DriveWeight {
    BooleanSupplier rightSensor;
    BooleanSupplier leftSensor;
    public ClimberAlignWeight(BooleanSupplier rightSensor, BooleanSupplier leftSensor){
        this.rightSensor = rightSensor;
        this.leftSensor = leftSensor;
    }

    @Override
    public ChassisSpeeds getSpeeds(){
        if (!rightSensor.getAsBoolean() && !leftSensor.getAsBoolean()){ // if neither triggered, go backwards
            return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(-0.01, 0,0), new Rotation2d(0));
        } else if (!rightSensor.getAsBoolean() && leftSensor.getAsBoolean()){ // if left is triggered and right is not, rotate the right side back by rotating clockwise
            return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(0, 0, 0.05), new Rotation2d(0));
        } else if (rightSensor.getAsBoolean() && !leftSensor.getAsBoolean()){
            return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(0, 0, -0.05), new Rotation2d(0));
        } else {
            return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(0, 0,0), new Rotation2d(0));
        }
    }
    @Override
    public boolean cancelCondition() {
        return (rightSensor.getAsBoolean() && leftSensor.getAsBoolean()); // both are triggered
    }
}
