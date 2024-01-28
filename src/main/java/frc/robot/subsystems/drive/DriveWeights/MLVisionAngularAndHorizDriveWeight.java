package frc.robot.subsystems.drive.DriveWeights;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.sensors.Vision.MLVision;

public class MLVisionAngularAndHorizDriveWeight implements DriveWeight {
    
    PIDController angularController = new PIDController(0.01, 0, 0);
    
    double angularVelocity;
    double verticalVelocity;
    MLVision vision;
    double deadband = 0; //0.1;
    double distanceLim = 0;
    ChassisSpeeds chassisSpeedsToTurn = new ChassisSpeeds(0,0,0);

    double calculatedEndTime = 0;


    public MLVisionAngularAndHorizDriveWeight(MLVision vision) {
        this.vision = vision;
    }

    @Override
    public ChassisSpeeds getSpeeds() {
       
        angularVelocity = angularController.calculate(vision.getTX());
        angularVelocity = (Math.abs(angularVelocity) < deadband) ? 0 : angularVelocity;
        angularVelocity = MathUtil.clamp(angularVelocity, -1, 1);

        //verticalVelocity = verticalController.calculate((vision.getDistance()) * 100); // cant be ty uh
        //verticalVelocity = (Math.abs(verticalVelocity) < deadband) ? 0 : verticalVelocity;
        verticalVelocity = 0.5; // ADD CONSTANT
        
        //if (!vision.isTarget()){
            //chassisSpeedsToTurn = new ChassisSpeeds(0,0,0);
            //return chassisSpeedsToTurn;
        //}

        //if (Math.abs(vision.getTX()) > distanceLim ){
            chassisSpeedsToTurn = new ChassisSpeeds(0,0,angularVelocity);
            return chassisSpeedsToTurn;    
       // else if (!isDriveToNoteFinished()) {          
            //chassisSpeedsToTurn = new ChassisSpeeds(angularVelocity,verticalVelocity,0);
            //return chassisSpeedsToTurn;        }
        
        //chassisSpeedsToTurn = new ChassisSpeeds(0,0,0);
        //return chassisSpeedsToTurn;     
        
    }
    
    
    public boolean isDriveToNoteFinished() { // add intake limit later
        
        if (vision.isTarget()){
            return false;
        }
        else if(!vision.isTarget()){
            if (calculatedEndTime == 0){
               //lostNoteVisualTime = Timer.getFPGATimestamp();
               calculatedEndTime = Timer.getFPGATimestamp() + 0.5; // idk makes the program run 3 more seconds to intake... probs change when we have a sensor in the intake
               return false;
            }
            else if(Timer.getFPGATimestamp() <= calculatedEndTime){
                return false;
            }
            return true;
        }
        return true;
    }


}
