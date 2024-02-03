package frc.robot.subsystems.drive.DriveWeights;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.sensors.Vision.MLVision;
import frc.robot.subsystems.drive.DriveWeightCommand;

public class MLVisionAngularAndHorizDriveWeight implements DriveWeight {
    
    PIDController angularController = new PIDController(0.0075, 0, 0); //Constants.PIDConstants.rotPID;
    PIDController horizontalController = new PIDController(0.008, 0, 0); //Constants.PIDConstants.rotPID;

    double angularVelocity;
    private double horizontalVelocity;
    double verticalVelocity;
    MLVision vision;
    private Supplier<Rotation2d> angleSupplier;
    //private Supplier<Rotation2d> correctedAngleSupplier;

    double deadband = 0; //0.1;
    double distanceLim = 10;
    ChassisSpeeds chassisSpeedsToTurn = new ChassisSpeeds(0,0,0);

    double initTime = 0;


    public MLVisionAngularAndHorizDriveWeight(MLVision vision, Supplier<Rotation2d> angleSupplier) {
        this.vision = vision;
        this.angleSupplier = angleSupplier;
    }

    @Override
    public ChassisSpeeds getSpeeds() {
        angularVelocity = angularController.calculate(vision.getTX());
        angularVelocity = (Math.abs(angularVelocity) < deadband) ? 0 : angularVelocity;
        angularVelocity = MathUtil.clamp(angularVelocity, -1, 1);

        horizontalVelocity = horizontalController.calculate(vision.getTX());
        horizontalVelocity = (Math.abs(horizontalVelocity) < deadband) ? 0 : horizontalVelocity;
        horizontalVelocity = MathUtil.clamp(horizontalVelocity, -1, 1);



        //verticalVelocity = verticalController.calculate((vision.getDistance()) * 100); // cant be ty uh
        //verticalVelocity = (Math.abs(verticalVelocity) < deadband) ? 0 : verticalVelocity;
        verticalVelocity = 0.2; // ADD CONSTANT
        
        if (!vision.isTarget() ){ //|| vision.getTA() < 2.7
            chassisSpeedsToTurn = new ChassisSpeeds(0,0,0);
            return chassisSpeedsToTurn;
        }

        else if (Math.abs(vision.getTX()) > distanceLim ){
            chassisSpeedsToTurn = new ChassisSpeeds(0,0, angularVelocity);
            return chassisSpeedsToTurn;
        }    
        else if (!isDriveToNoteFinished()) {          
            chassisSpeedsToTurn = ChassisSpeeds.fromRobotRelativeSpeeds(
                new ChassisSpeeds(-verticalVelocity, -horizontalVelocity, 0),
                angleSupplier.get()
            );
            return chassisSpeedsToTurn;
        }
        
        return chassisSpeedsToTurn;
    }
    
    
    public boolean isDriveToNoteFinished() { // add intake limit later
        
        if (vision.isTarget()){
            return false;
        }
        else if(!vision.isTarget()){
            if (initTime == 0){
                initTime = System.currentTimeMillis();
                return false;
            }

            if (initTime + 500 > System.currentTimeMillis()){
                initTime = 0;
                return true;
            }

        }
        return false;
    }


}