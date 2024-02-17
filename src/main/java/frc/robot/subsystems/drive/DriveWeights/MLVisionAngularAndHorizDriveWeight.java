package frc.robot.subsystems.drive.DriveWeights;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.PIDConstants;
import frc.robot.sensors.Vision.MLVision.MLVision;


public class MLVisionAngularAndHorizDriveWeight implements DriveWeight {
    
    private PIDController angularController = PIDConstants.constructPID(PIDConstants.rotMLVision); //;
    private PIDController horizontalController = PIDConstants.constructPID(PIDConstants.horizontalMLVision); //Constants.PIDConstants.rotPID;

    private double angularVelocity;
    private double horizontalVelocity;
    private double verticalVelocity;
    private MLVision vision;
    private Supplier<Rotation2d> angleSupplier;
    //private Supplier<Rotation2d> correctedAngleSupplier;

    private double deadband = 0; //0.1;
    private double distanceLim = 10;
    private ChassisSpeeds chassisSpeedsToTurn = new ChassisSpeeds(0,0,0);

    private double initTime = 0;


    public MLVisionAngularAndHorizDriveWeight(MLVision vision,Supplier<Rotation2d> angleSupplier) {
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
        verticalVelocity = 0.2; // ADD CONSTANT
        
        if (!vision.isTarget() ){ 
            chassisSpeedsToTurn = new ChassisSpeeds(0,0,0);
        }

        else if (Math.abs(vision.getTX()) > distanceLim ){
            chassisSpeedsToTurn = new ChassisSpeeds(0,0, angularVelocity);
        }    
        else if (!isDriveToNoteFinished()) {          
            chassisSpeedsToTurn = ChassisSpeeds.fromRobotRelativeSpeeds(
                new ChassisSpeeds(-verticalVelocity, -horizontalVelocity, 0),
                angleSupplier.get()
            );
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