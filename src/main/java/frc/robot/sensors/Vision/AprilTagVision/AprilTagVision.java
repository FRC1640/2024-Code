package frc.robot.sensors.Vision.AprilTagVision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.periodic.PeriodicBase;
import frc.robot.Constants.FieldConstants;

public class AprilTagVision extends PeriodicBase {
    private AprilTagVisionIO io;
    private AprilTagVisionIOInputsAutoLogged inputs = new AprilTagVisionIOInputsAutoLogged();
    private String key;

    public AprilTagVision(AprilTagVisionIO io, String key) {
        this.io = io;
        this.key = key;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("AprilTagVision" + key, inputs);
    }

    public int getNumVisibleTags(){
        return inputs.numVisibleTags;
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

    public double getTa(){
        return inputs.ta;
    }

    public boolean isPoseValid(Pose2d pose){
        return FieldConstants.width >= pose.getX() && 
        FieldConstants.height >= pose.getY() && pose.getX() >= 0 && pose.getY() >= 0 && pose.getTranslation().getX() != 0;
    }
}
