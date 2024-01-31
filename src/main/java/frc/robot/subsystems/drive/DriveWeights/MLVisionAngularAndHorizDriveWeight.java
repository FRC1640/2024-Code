package frc.robot.subsystems.drive.DriveWeights;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.sensors.Vision.MLVision;
import frc.robot.subsystems.drive.DriveWeightCommand;

public class MLVisionAngularAndHorizDriveWeight implements DriveWeight {
    
    PIDController angularController = new PIDController(0.01, 0, 0);
    
    double angularVelocity;
    double verticalVelocity;
    MLVision vision;
    double deadband = 0; //0.1;
    double distanceLim = 10;
    ChassisSpeeds chassisSpeedsToTurn = new ChassisSpeeds(0,0,0);

    double initTime = 0;


    public MLVisionAngularAndHorizDriveWeight(MLVision vision) {
        this.vision = vision;
    }

    @Override
    public ChassisSpeeds getSpeeds() {
        return new ChassisSpeeds(0,0,0);}
/* 
        System.out.println("TX: " + vision.getTX());
       
        angularVelocity = angularController.calculate(vision.getTX());
        angularVelocity = (Math.abs(angularVelocity) < deadband) ? 0 : angularVelocity;
        angularVelocity = MathUtil.clamp(angularVelocity, -1, 1);

        //verticalVelocity = verticalController.calculate((vision.getDistance()) * 100); // cant be ty uh
        //verticalVelocity = (Math.abs(verticalVelocity) < deadband) ? 0 : verticalVelocity;
        verticalVelocity = 0.08; // ADD CONSTANT
        
        if (!vision.isTarget()){
            chassisSpeedsToTurn = new ChassisSpeeds(0,0,0);
            //System.out.println("1");
            return chassisSpeedsToTurn;
        }

        else if (Math.abs(vision.getTX()) > distanceLim ){
            //System.out.println("2");
            chassisSpeedsToTurn = new ChassisSpeeds(0,0,angularVelocity);
            return chassisSpeedsToTurn;
        }    
        else if (!isDriveToNoteFinished()) {          
            chassisSpeedsToTurn = new ChassisSpeeds(0,angularVelocity, 0);  
            //System.out.println("3");      
            //System.out.println("lowkey moving forward");
            return chassisSpeedsToTurn;        
    
        } 
        else{
            //chassisSpeedsToTurn = new ChassisSpeeds(0,0,0);
            System.out.println("4");
            DriveWeightCommand.removeWeight(this);
        }
        System.out.println("5");
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

 */
}
