package frc.robot.sensors.Vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.periodic.PeriodicBase;

public class Vision extends PeriodicBase {
    private VisionIO io;
    private VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    public Vision(VisionIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);
    }

    public Pose2d getAprilTagPose2d(){
        return inputs.aprilTagPose;
    }
    public double getLatency(){
        return inputs.latency;
    }
    public boolean isTarget(){
        return inputs.isTarget;
    }
}
