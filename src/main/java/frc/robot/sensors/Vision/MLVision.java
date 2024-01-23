package frc.robot.sensors.Vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.periodic.PeriodicBase;
import frc.robot.Constants;

public class MLVision extends PeriodicBase {
    private MLVisionIO io;
    private MLVisionIOInputsAutoLogged inputs = new MLVisionIOInputsAutoLogged();
    private double trigDistance;

    private double isFinishedTolerance;
 
    private PIDController angController;
    private double angularVelocity;

    private PIDController verticalPIDController;
    private double verticalVelocity;
 
    private double kP; // PID constant
    private double deadband; // velocity deadband


    public MLVision(MLVisionIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ML Vision", inputs);
    }


    // Getters 

    public double getLatency(){
        return inputs.latency;
    }
    public boolean isTarget(){
        return inputs.isTarget;
    }

    public double getTX(){
        return inputs.tx; // just... hope for point of interest tracking
    }
    
    public double getTY(){
        return inputs.ty;
    }
    
    public double getTA(){
        return inputs.ta;
    }

    public double getDistance(){
        // Using data points and trigonometry for distance calculations from the
        // Limelight to the object detected.
        
        trigDistance = Units.inchesToMeters( // "d = (h2-h1) / tan(a1+a2)"
                (Constants.LimelightConstants.noteHeightCentimers - Constants.LimelightConstants.limelightLensHeight)
                        / Math.tan(Math.toRadians(inputs.ta + Constants.LimelightConstants.limelightAngle)));
        
        return trigDistance;
    }
  

}