//why did i do it like this...
package frc.robot.sensors.Vision.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.sensors.Vision.MLVision;
import frc.robot.subsystems.drive.DriveSubsystem;

public class VerticalAlignToNoteCommand extends Command{
    private DriveSubsystem driveSubsystem;
    private MLVision MLVision;
    
    //private double lostNoteVisualTime = 0;
    private double calculatedEndTime = 0;

    private PIDController verticalPIDController;
    private double verticalVelocity;
    
    private double kP; // PID constant
     private double deadband; // velocity deadband


    public VerticalAlignToNoteCommand(DriveSubsystem driveSubsystem, MLVision MLVision) {
        
        this.driveSubsystem = driveSubsystem;
        this.MLVision = MLVision;

        addRequirements(driveSubsystem);
    }

    public void initialize() {
        verticalPIDController = new PIDController(kP, 0, 0);
    }

    public void execute() {
        verticalVelocity = verticalPIDController.calculate((MLVision.getDistance()) * 100); // cant be ty uh
        verticalVelocity = (Math.abs(verticalVelocity) < deadband) ? 0 : verticalVelocity;
        driveSubsystem.drivePercentDoubleCone(0, verticalVelocity, 0, false);
    }

    public boolean isFinished() {
        
        if (MLVision.isTarget()){
            return false;
        }
        else if(!MLVision.isTarget()){
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
    
}
