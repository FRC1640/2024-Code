package frc.robot.sensors.Vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.periodic.PeriodicBase;
import frc.robot.Constants;

public class MLVision extends PeriodicBase {
    private MLVisionIO io;
    private MLVisionIOInputsAutoLogged inputs = new MLVisionIOInputsAutoLogged();
    private double trigDistance;


    public MLVision(MLVisionIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ML Vision", inputs);
    }

    public double getLatency(){
        return inputs.latency;
    }
    public boolean isTarget(){
        return inputs.isTarget;
    }

    public double getTX(){
        return inputs.tx;
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