package frc.robot.subsystems.drive.DriveWeights;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import frc.lib.drive.DriveWeight;
import frc.robot.Constants.SwerveDriveDimensions;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ClimberAlignWeight implements DriveWeight {
    private BooleanSupplier rightSensor;
    private BooleanSupplier leftSensor;
    private Translation2d centerOfRot;
    private Supplier<Rotation2d> angleSupplier;

    public ClimberAlignWeight(BooleanSupplier rightSensor, BooleanSupplier leftSensor, Supplier<Rotation2d> angleSupplier){
        this.rightSensor = rightSensor;
        this.leftSensor = leftSensor;
        this.angleSupplier = angleSupplier;
    }

    @Override
    public ChassisSpeeds getSpeeds(){
        if (!rightSensor.getAsBoolean() && !leftSensor.getAsBoolean()){ // if neither triggered, go backwards
            centerOfRot = new Translation2d();
            return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(-0.1, 0,0), angleSupplier.get());
        } else if (!rightSensor.getAsBoolean() && leftSensor.getAsBoolean()){ // if left is triggered and right is not, rotate the right side back by rotating clockwise
            centerOfRot = SwerveDriveDimensions.backLeftLocation;
            return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(0, 0, -0.05),  angleSupplier.get());
        } else if (rightSensor.getAsBoolean() && !leftSensor.getAsBoolean()){
            centerOfRot = SwerveDriveDimensions.backRightLocation;
            return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(0, 0, 0.05), angleSupplier.get());
        } else {
            return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(0, 0,0), angleSupplier.get());
        }
    }

    @Override
    public Translation2d getCenterOfRot(){
        return centerOfRot;
    }

    @Override
    public boolean cancelCondition() {
        return (rightSensor.getAsBoolean() && leftSensor.getAsBoolean()); // both are triggered
    }

    

}
