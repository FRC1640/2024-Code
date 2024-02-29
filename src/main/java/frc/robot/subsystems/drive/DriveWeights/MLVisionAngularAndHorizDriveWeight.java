package frc.robot.subsystems.drive.DriveWeights;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.sensors.Vision.MLVision.MLVision;

public class MLVisionAngularAndHorizDriveWeight implements DriveWeight {

    private PIDController angularController = new PIDController(0.006, 0, 0); // Constants.PIDConstants.rotPID;
    private PIDController horizontalController = new PIDController(0.006, 0, 0); // Constants.PIDConstants.rotPID;

    private double angularVelocity = 0;
    private double horizontalVelocity = 0;
    
    private double verticalVelocity = 0; // ADD CONSTANT

    private MLVision vision;
    private Supplier<Rotation2d> angleSupplier;
    // private Supplier<Rotation2d> correctedAngleSupplier;

    private double deadband = 0; // 0.1;
    private double distanceLim = 10;
    private ChassisSpeeds chassisSpeedsToTurn = new ChassisSpeeds(0, 0, 0);

    private double initTime = 0;

    private boolean rotateMode = true;
    

    public MLVisionAngularAndHorizDriveWeight(MLVision vision, Supplier<Rotation2d> angleSupplier) {
        this.vision = vision;
        this.angleSupplier = angleSupplier;

    }

    @Override
    public ChassisSpeeds getSpeeds() {
        horizontalVelocity = 0;
        verticalVelocity = 0;
        angularVelocity = 0;

        if (!vision.isTarget()) {
            //System.out.println(" NO Target " + vision.isTarget() + targetNoteSet);
            horizontalVelocity = 0;
            verticalVelocity = 0;
            angularVelocity = 0;
            chassisSpeedsToTurn = 
            new ChassisSpeeds(0 , 0, 0); 

        }
        else if (Math.abs(vision.getTX()) > distanceLim && rotateMode) {
            horizontalVelocity = 0;
            
            verticalVelocity = 0;
           
            angularVelocity = angularController.calculate(vision.getTX());
            angularVelocity = (Math.abs(angularVelocity) < deadband) ? 0 : angularVelocity;
            angularVelocity = MathUtil.clamp(angularVelocity, -1, 1);

            chassisSpeedsToTurn = new ChassisSpeeds(0 , 0, angularVelocity); 

            //deltaTX = vision.getTX() - previousTX;
            //Logger.recordOutput("MLVision/Delta TX", deltaTX);
            
        }
        else {
            rotateMode = false;

            horizontalVelocity = horizontalController.calculate(vision.getTX());
            horizontalVelocity = (Math.abs(horizontalVelocity) < deadband) ? 0 : horizontalVelocity;
            horizontalVelocity = MathUtil.clamp(horizontalVelocity, -1, 1);
            
            //verticalVelocity = verticalVelocityConstant;
            verticalVelocity = 0.4;

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
}