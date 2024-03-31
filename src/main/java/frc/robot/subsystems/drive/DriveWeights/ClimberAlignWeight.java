package frc.robot.subsystems.drive.DriveWeights;

import java.util.function.BooleanSupplier;
import frc.robot.Constants.SwerveDriveDimensions;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class ClimberAlignWeight implements DriveWeight {
    private BooleanSupplier rightSensor;
    private BooleanSupplier leftSensor;
    private Translation2d centerOfRot;

    public ClimberAlignWeight(BooleanSupplier rightSensor, BooleanSupplier leftSensor){
        this.rightSensor = rightSensor;
        this.leftSensor = leftSensor;
    }

    @Override
    public ChassisSpeeds getSpeeds(){
        if (!rightSensor.getAsBoolean() && !leftSensor.getAsBoolean()){ // if neither triggered, go backwards
            centerOfRot = new Translation2d();
            return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(-0.01, 0,0), new Rotation2d(0));
        } else if (!rightSensor.getAsBoolean() && leftSensor.getAsBoolean()){ // if left is triggered and right is not, rotate the right side back by rotating clockwise
            centerOfRot = SwerveDriveDimensions.backLeftLocation;
            return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(0, 0, -0.05),  new Rotation2d());
        } else if (rightSensor.getAsBoolean() && !leftSensor.getAsBoolean()){
            centerOfRot = SwerveDriveDimensions.backRightLocation;
            return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(0, 0, 0.05), new Rotation2d(0));
        } else {
            return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(0, 0,0), new Rotation2d());
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
