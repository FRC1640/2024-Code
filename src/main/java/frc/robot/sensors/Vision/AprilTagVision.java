package frc.robot.sensors.Vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.periodic.PeriodicBase;
import frc.robot.Constants.FieldConstants;

public class AprilTagVision extends PeriodicBase {
    private AprilTagVisionIO io;
    private AprilTagVisionIOInputsAutoLogged inputs = new AprilTagVisionIOInputsAutoLogged();

    public AprilTagVision(AprilTagVisionIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("AprilTagVision", inputs);
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
    public double getDistance(){
        return inputs.aprilTagDistance;
    }

    public boolean isPoseValid(Pose2d pose){
        return FieldConstants.width >= pose.getX() && FieldConstants.height >= pose.getY() && pose.getX() >= 0 && pose.getY() >= 0;
    }
}
