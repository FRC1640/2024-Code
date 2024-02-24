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
    private double distanceLim = 4; // angular tx disparity deadband idk if thats what i should call it

    private ChassisSpeeds chassisSpeedsToTurn = new ChassisSpeeds(0, 0, 0);
    
    private double timeOutMillisecs = 200;

    private double initTime = -1;
    private double initIntakeModeTime = 0; // initialize drive straight until intookith or timithed out 
    private double previousTY;
    private double deltaTYlim = 1; // if delta ty > than this, enter drive straight to intake mode

    private boolean intakeMode;
    private boolean rotateMode;
    

    public MLVisionAngularAndHorizDriveWeight(MLVision vision, Supplier<Rotation2d> angleSupplier) {
        this.vision = vision;
        this.angleSupplier = angleSupplier;
        
        intakeMode = false;
        rotateMode = true;
        previousTY = -1;

        initTime = -1;
        initIntakeModeTime = 0;

    }

    @Override
    public ChassisSpeeds getSpeeds() {
        horizontalVelocity = 0;
        verticalVelocity = 0;
        angularVelocity = 0;

        if (!intakeMode){
            determineIntakeNoteMode();
        }

        if (intakeMode){
            verticalVelocity = 0.1;
            chassisSpeedsToTurn = ChassisSpeeds.fromRobotRelativeSpeeds(
                new ChassisSpeeds(-verticalVelocity, 0, 0),
                angleSupplier.get()); 
            return chassisSpeedsToTurn; 
        }


        if (!vision.isTarget() || (System.currentTimeMillis()> initIntakeModeTime + timeOutMillisecs && initIntakeModeTime != 0) ) { // If no target is visible, return no weight
            //System.out.println(" NO Target " + vision.isTarget() + targetNoteSet);
            horizontalVelocity = 0;
            verticalVelocity = 0;
            angularVelocity = 0;
            chassisSpeedsToTurn = 
            new ChassisSpeeds(0 , 0, 0); 

        }
        else if (Math.abs(vision.getTX()) > distanceLim && rotateMode) { // if the target tx is within the accepted tx range AND the the weight has never exited rotate mode, JUST rotate
            horizontalVelocity = 0;
            
            verticalVelocity = 0;
           
            angularVelocity = angularController.calculate(vision.getTX());
            angularVelocity = (Math.abs(angularVelocity) < deadband) ? 0 : angularVelocity;
            angularVelocity = MathUtil.clamp(angularVelocity, -1, 1);

            chassisSpeedsToTurn = new ChassisSpeeds(0 , 0, angularVelocity); 

            //deltaTX = vision.getTX() - previousTX;
            //Logger.recordOutput("MLVision/Delta TX", deltaTX);
            
        }
        else{ // enter horrizorntal strafe mode
            rotateMode = false;
            previousTY = vision.getTY();


            horizontalVelocity = horizontalController.calculate(vision.getTX());
            horizontalVelocity = (Math.abs(horizontalVelocity) < deadband) ? 0 : horizontalVelocity;
            horizontalVelocity = MathUtil.clamp(horizontalVelocity, -1, 1);
            
            //verticalVelocity = verticalVelocityConstant;
            verticalVelocity = 0.2;

            angularVelocity = 0;
            
            chassisSpeedsToTurn = ChassisSpeeds.fromRobotRelativeSpeeds(
            new ChassisSpeeds(-verticalVelocity, -horizontalVelocity, 0),
             angleSupplier.get()); 
        }

        
     
        Logger.recordOutput("MLVision/Input Rotational Velocity", angularVelocity);
        Logger.recordOutput("MLVision/Input Horizontal Velocity", horizontalVelocity);
        Logger.recordOutput("MLVision/Input Vertical Velocity", verticalVelocity);
     
        
        return chassisSpeedsToTurn;
    }

    public void resetMode(){
        intakeMode = false;
        rotateMode = true;
        previousTY = -1;

        initTime = -1;
        initIntakeModeTime = 0;
    }


    private boolean isDriveToNoteFinished() { // add intake limit later

        if (vision.isTarget()) {
            return false;
        } else if (!vision.isTarget()) {
            if (initTime == -1) {
                initTime = System.currentTimeMillis();
                return false;
            }

            if (initTime + 500 > System.currentTimeMillis()) {
                initTime = 0;
                return true;
            }

        }
        return false;
    }

    private void determineIntakeNoteMode(){
        if (Math.abs(previousTY - vision.getTY()) > deltaTYlim && (previousTY < -20)){ // IF the ty makes a big jump and it used to be small
            initIntakeModeTime = System.currentTimeMillis(); // enter intake mode
            intakeMode = true;
        }
        else if (System.currentTimeMillis() < initIntakeModeTime + timeOutMillisecs) { // if the current time is less than half a second away  
            intakeMode = true;        
        } 
        else{
            intakeMode= false;
    }




    }
}