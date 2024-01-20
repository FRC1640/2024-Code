package frc.robot.sensors.Vision.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.sensors.Vision.MLVision;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AngularAlignToNoteCommand extends Command{
 private DriveSubsystem driveSubsystem;
 private MLVision MLVision;
 private double isFinishedTolerance;
 
 private PIDController angController;
 private double angularVelocity;
 
 private double kP; // PID constant
  private double deadband; // velocity deadband



// ROTATE TO CENTER THE NOTE
    public AngularAlignToNoteCommand(DriveSubsystem driveSubsystem, MLVision MLVision) {
        
        this.driveSubsystem = driveSubsystem;
        this.MLVision = MLVision;

        addRequirements(driveSubsystem);
    }
   
    public void initialize() {
        angController = new PIDController(kP, 0, 0);
    }

    public void execute() {
        angularVelocity = angController.calculate(MLVision.getTX());
        angularVelocity = (Math.abs(angularVelocity) < deadband) ? 0 : angularVelocity;
        driveSubsystem.drivePercentDoubleCone(0, 0, angularVelocity, false);
    }

    public boolean isFinished() {
        if (Math.abs(MLVision.getTX()) > isFinishedTolerance) { // GONNA HAVE TO CONSIDER LIMELIGHT X ON THE ROBOT SOMEWHERE...
            return false;
        }
        return true;
    }
}
