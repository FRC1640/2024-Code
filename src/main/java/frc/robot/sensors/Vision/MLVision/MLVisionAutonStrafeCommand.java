package frc.robot.sensors.Vision.MLVision;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.drive.DriveSubsystem;
import frc.robot.Constants.PIDConstants;

public class MLVisionAutonStrafeCommand {
    private PIDController horizontalController = PIDConstants.constructPID(PIDConstants.horizontalMLVision, "mldrive"); //Constants.PIDConstants.rotPID;

    private double horizontalVelocity = 0;
    private double verticalVelocity = 0.4;

    private MLVision vision;
    private Supplier<Rotation2d> angleSupplier; // current angle (for us to offset the chassis speed angular position to drive straight)

    private double deadband = 0; // 0.1;
    private double distanceLim = 1.5; // angular tx disparity deadband idk if thats what i should call it

    private ChassisSpeeds chassisSpeedsToTurn = new ChassisSpeeds(0, 0, 0); // this is, in fact, the chassis speed we will be using to turn
    
    //private double timeOutMillisecs = 200;

    private double initTime = -1;

    private double previousTA;
    private double deltaTAlim = 2; // if delta ty > than this, enter drive straight to intake mode

    private double previousTY;
    private double previousTYlim = -5;
    
    private boolean intakeMode;

    private DriveSubsystem driveSubsystem;

    private Supplier<Boolean> hasNote;

    public MLVisionAutonStrafeCommand(MLVision vision, Supplier<Rotation2d> angleSupplier, DriveSubsystem driveSubsystem, Supplier<Boolean> hasNote){
        this.vision = vision;
        this.angleSupplier = angleSupplier;
        
        this.driveSubsystem = driveSubsystem;

        this.hasNote = hasNote; 

        intakeMode = false;
        
        previousTA = vision.getTA();
        previousTY = vision.getTY();
        initTime = System.currentTimeMillis();

    }

    public Command getCommand(){
        return new InstantCommand(()->resetMode()).andThen(driveSubsystem.driveDoubleConeCommand(()->getSpeeds(), ()->new Translation2d()).until(()->isFinished()).repeatedly());
    }


    public boolean isFinished(){
        return hasNote.get() || System.currentTimeMillis() - initTime > 5000;
    }

    public ChassisSpeeds getSpeeds() {
        horizontalVelocity = 0;

        if (!intakeMode){
            
            determineIntakeNoteMode();

            if (!vision.isTarget()) { // If no target is visible, return no weight                
                chassisSpeedsToTurn = 
                    new ChassisSpeeds(0 , 0, 0); 

            }
            else{ // Otherwise enter horrizorntal strafe mode
                previousTA = vision.getTA();
                previousTY = vision.getTY();

                horizontalVelocity = horizontalController.calculate(vision.getTX());
                horizontalVelocity = (Math.abs(horizontalVelocity) < deadband) ? 0 : horizontalVelocity;
                horizontalVelocity = MathUtil.clamp(horizontalVelocity, -1, 1);

            
                chassisSpeedsToTurn = ChassisSpeeds.fromRobotRelativeSpeeds(
                    new ChassisSpeeds(0, -horizontalVelocity, 0),
                    angleSupplier.get()); 
            }
        }
        else { // If the robot is IN intake mode
            chassisSpeedsToTurn = ChassisSpeeds.fromRobotRelativeSpeeds(
                    new ChassisSpeeds(-verticalVelocity, 0, 0),
                    angleSupplier.get()); 
        }

        
        return chassisSpeedsToTurn;
    }

    private void determineIntakeNoteMode(){

        intakeMode = previousTA - vision.getTA() > deltaTAlim 
            && previousTY < previousTYlim ||  (Math.abs(vision.getTX()) < distanceLim);// IF the ta makes a big jump and it used to be small
    }
    
    public void resetMode(){
        intakeMode = false;
        previousTA = vision.getTA();
        previousTY = vision.getTY();


        initTime = System.currentTimeMillis();
    }
}
