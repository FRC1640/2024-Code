package frc.robot.sensors.Vision.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.drive.DriveSubsystem;
import frc.robot.sensors.Vision.MLVision;


public class MLVisionFactory {
    DriveSubsystem driveSubsystem;
    MLVision vision;
     
    private double isFinishedTolerance;
    private double kP; // PID constant
    private double deadband; // velocity deadband
 

    private PIDController angularController = new PIDController(0, 0, 0); // TODO switch to constants
    private double angularVelocity;

    private PIDController verticalPIDController = new PIDController(0, 0, 0); // TODO switch to constants
    private double verticalVelocity;

     private double calculatedEndTime = 0;


    double kp;

    public MLVisionFactory(){

    }
   
    //Align to Note Command
    public ChassisSpeeds calculateAngularSpeeds(){

        angularVelocity = angularController.calculate(vision.getTX());
        angularVelocity = (Math.abs(angularVelocity) < deadband) ? 0 : angularVelocity;
        angularVelocity = MathUtil.clamp(angularVelocity, -1, 1);


        ChassisSpeeds chassisSpeedsToTurn = new ChassisSpeeds(0, 0, angularVelocity);
        return chassisSpeedsToTurn;
    }

    public Command angularAlignToNoteCommand() {
        return new RunCommand(()->driveSubsystem.driveDoubleConeCommand(()-> calculateAngularSpeeds()).until(()->Math.abs(vision.getTX()) <= isFinishedTolerance));
    }
    
    //Align to Note Command
    private ChassisSpeeds calculateVerticalSpeeds(){

        verticalVelocity = verticalPIDController.calculate((vision.getDistance()) * 100); // cant be ty uh
        verticalVelocity = (Math.abs(verticalVelocity) < deadband) ? 0 : verticalVelocity;
        verticalVelocity = 0; // ADD CONSTANTS

        ChassisSpeeds chassisSpeedsToDrive = new ChassisSpeeds(0, verticalVelocity, 0);
        return chassisSpeedsToDrive;
    }

    public boolean isDriveToNoteFinished() { // add intake limit later
        
        if (vision.isTarget()){
            return false;
        }
        else if(!vision.isTarget()){
            if (calculatedEndTime == 0){
               //lostNoteVisualTime = Timer.getFPGATimestamp();
               calculatedEndTime = Timer.getFPGATimestamp() + 3; // idk makes the program run 3 more seconds to intake... probs change when we have a sensor in the intake
               return false;
            }
            else if(Timer.getFPGATimestamp() <= calculatedEndTime){
                return false;
            }
            return true;
        }
        return true;
    }
    
    private Command driveToNoteCommand() {
        return new RunCommand(()->driveSubsystem.driveDoubleConeCommand(()-> calculateVerticalSpeeds())
            .until(()->isDriveToNoteFinished()));
    }

    public Command angleAndDriveToNoteCommand(){
        SequentialCommandGroup command = new SequentialCommandGroup(angularAlignToNoteCommand(), driveToNoteCommand());
        return command;
    }

  
    
}
