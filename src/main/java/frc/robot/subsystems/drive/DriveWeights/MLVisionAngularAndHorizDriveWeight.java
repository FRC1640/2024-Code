package frc.robot.subsystems.drive.DriveWeights;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.PIDConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.sensors.Vision.MLVision.MLVision;

public class MLVisionAngularAndHorizDriveWeight implements DriveWeight {

    //private PIDController angularController = new PIDController(0.006, 0, 0); // Constants.PIDConstants.rotPID;
    //private PIDController horizontalController = new PIDController(0.006, 0, 0); // Constants.PIDConstants.rotPID;
    private PIDController angularController = PIDConstants.constructPID(PIDConstants.rotMLVision, "mlrot"); //;
    private PIDController horizontalController = PIDConstants.constructPID(PIDConstants.horizontalMLVision, "mldrive"); //Constants.PIDConstants.rotPID;

    private double angularVelocity = 0;
    private double horizontalVelocity = 0;
    
    private double verticalVelocity = 0; // ADD CONSTANT

    private MLVision vision;
    private Supplier<Rotation2d> angleSupplier;
    // private Supplier<Rotation2d> correctedAngleSupplier;

    private double deadband = 0; // 0.1;
    private double distanceLim = 3; // angular tx disparity deadband idk if thats what i should call it

    private ChassisSpeeds chassisSpeedsToTurn = new ChassisSpeeds(0, 0, 0);
    
    private double timeOutMillisecs = 200;

    private double initIntakeModeTime = -1; // initialize drive straight until intookith or timithed out 

    private double previousTA;
    private double deltaTAlim = 2; // if delta ty > than this, enter drive straight to intake mode

    private double previousTY;
    private double previousTYlim = -5;
    
    private boolean intakeMode;
    private boolean rotateMode;

    private Supplier<Boolean> hasNote;
    

    public MLVisionAngularAndHorizDriveWeight(MLVision vision, Supplier<Rotation2d> angleSupplier) { //Supplier<Boolean> hasNote
        this.vision = vision;
        this.angleSupplier = angleSupplier;
        
        intakeMode = false;
        rotateMode = true;
        
        previousTA = vision.getTA();
        previousTY = vision.getTY();

        initIntakeModeTime = -1;

        //this.hasNote = hasNote;
    }

    @Override
    public ChassisSpeeds getSpeeds() {
        horizontalVelocity = 0;
        verticalVelocity = 0;
        angularVelocity = 0;

       // if (hasNote.get()){
            //return new ChassisSpeeds(0 , 0, 0); 
        //}

        if (!intakeMode){
            
            determineIntakeNoteMode();

            if (!vision.isTarget()) { // If no target is visible, return no weight                
                chassisSpeedsToTurn = 
                    new ChassisSpeeds(0 , 0, 0); 

            }
            else if (Math.abs(vision.getTX()) > distanceLim && rotateMode) { // if the target tx is within the accepted tx range AND the the weight has never exited rotate mode, JUST rotate
                horizontalVelocity = 0;
            
                verticalVelocity = 0;
           
                angularVelocity = angularController.calculate(vision.getTX());
                angularVelocity = (Math.abs(angularVelocity) < deadband) ? 0 : angularVelocity;
                angularVelocity = MathUtil.clamp(angularVelocity, -1, 1);

                chassisSpeedsToTurn = 
                    new ChassisSpeeds(0 , 0, angularVelocity); 
            }
            else{ // Otherwise enter horrizorntal strafe mode
                rotateMode = false;
                previousTA = vision.getTA();
                previousTY = vision.getTY();



                horizontalVelocity = horizontalController.calculate(vision.getTX());
                horizontalVelocity = (Math.abs(horizontalVelocity) < deadband) ? 0 : horizontalVelocity;
                horizontalVelocity = MathUtil.clamp(horizontalVelocity, -1, 1);
            
                //verticalVelocity = verticalVelocityConstant;
                verticalVelocity = 0.3;

                angularVelocity = 0;
            
                chassisSpeedsToTurn = ChassisSpeeds.fromRobotRelativeSpeeds(
                    new ChassisSpeeds(-verticalVelocity, -horizontalVelocity, 0),
                    angleSupplier.get()); 
            }
        }
        else { // If the robot is IN intake mode, just go straight
            if (!isDriveToNoteFinished()){
                verticalVelocity = 0.2;
                    chassisSpeedsToTurn = ChassisSpeeds.fromRobotRelativeSpeeds(
                        new ChassisSpeeds(-verticalVelocity, 0, 0),
                        angleSupplier.get()); 
            }
            else{
                chassisSpeedsToTurn = 
                    new ChassisSpeeds(0 , 0, 0); 
            }
        }

        
        return chassisSpeedsToTurn;
    }

    public void resetMode(){
        intakeMode = false;
        rotateMode = true;
        previousTA = vision.getTA();
        previousTY = vision.getTY();

        initIntakeModeTime = -1;
    }


    private boolean isDriveToNoteFinished() { // add intake limit later

        if (!intakeMode) {
            return false;
        } 
        else {
            if (initIntakeModeTime == -1) {
                initIntakeModeTime = System.currentTimeMillis();
                return false;
            }

            if (initIntakeModeTime + 200 < System.currentTimeMillis()) {
                return true;
            }

        }
        return false;
    }

    private void determineIntakeNoteMode(){
        if (previousTA - vision.getTA() > deltaTAlim && previousTY < previousTYlim){ // IF the ta makes a big jump and it used to be small
            initIntakeModeTime = System.currentTimeMillis(); // enter intake mode
            intakeMode = true;
            //initTime = System.currentTimeMillis();
        }
        else{
            intakeMode = false;
    }
    }

}