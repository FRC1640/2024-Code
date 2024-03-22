package frc.robot.sensors.Vision.MLVision;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.opencv.videoio.Videoio;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.drive.DriveSubsystem;
import frc.robot.Constants.PIDConstants;

public class MLVisionAutoCommand2 {
    private BooleanSupplier hasNote;
    private MLVision vision;
    double lastTX = 0;
    boolean horizontalPID;
    boolean canDrive;
    double initNoNoteTime;
    private PIDController horizontalController = PIDConstants.constructPID(PIDConstants.horizontalMLVision, "mldrive");
    private PIDController horizontalControllerDrive = PIDConstants.constructPID(PIDConstants.horizontalMLVisionDrive, "mldrive2");
    private DriveSubsystem driveSubsystem;
    private Supplier<Rotation2d> rotation;

    public MLVisionAutoCommand2(BooleanSupplier hasNote, MLVision vision, DriveSubsystem driveSubsystem, Supplier<Rotation2d> rotation){
        this.hasNote = hasNote;
        this.vision = vision;
        this.driveSubsystem = driveSubsystem;
        this.rotation = rotation;

    }

        public Command getCommand(){
        return new InstantCommand(()->init()).andThen(driveSubsystem.driveDoubleConeCommand(()->getSpeeds(), ()->new Translation2d()).repeatedly()).until(()->isFinished());
    }


    public boolean isFinished(){
        return hasNote.getAsBoolean() || System.currentTimeMillis() - initNoNoteTime > 2000;
    }


    public ChassisSpeeds getSpeeds(){
        if (vision.getTA() > 2.5 && vision.isTarget()){
            initNoNoteTime = System.currentTimeMillis();
        }
        if (vision.getTA() > 15 && vision.isTarget() && Math.abs(vision.getTX()) <= 5.5){
            canDrive = true;
        }
        if (vision.getTA() > 15 && vision.isTarget() && Math.abs(vision.getTX()) > 5.5){
            return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(0, -horizontalController.calculate(vision.getTX(), 0), 0),
            rotation.get());
        }
        if ((!vision.isTarget() || vision.getTA() < 2.5) && canDrive){
            return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(-0.4, 0, 0),rotation.get());
        }
        if (vision.getTA() >= 2.5 && vision.isTarget()){
            lastTX = vision.getTX();
            horizontalPID = true;
            return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(-0.3, -horizontalControllerDrive.calculate(vision.getTX(), 0), 0),rotation.get());
        }
        if (!vision.isTarget() && horizontalPID){
            
            return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(-0.3, 0.5 * -horizontalController.calculate(lastTX, 0), 0),rotation.get());
        }
        return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(0,0,0),rotation.get());
    }

    public void init(){
        initNoNoteTime = System.currentTimeMillis();
        lastTX = 0;
        horizontalPID = false;
        canDrive = false;
    }
    
}
