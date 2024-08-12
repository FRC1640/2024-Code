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

    public Pose2d getAprilTagPose2dMT2(){
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
    public double[] getDistances(){
        return inputs.aprilTagDistances;
    }

    public Pose2d getAprilTagPose2dMT1(){
        return inputs.aprilTagPoseRot;
    }

    public double getTa(){
        return inputs.ta;
    }

    public double getTx(){
        return inputs.tx;
    }

    public boolean isPoseValid(Pose2d pose){
        return FieldConstants.width >= pose.getX() && 
        FieldConstants.height >= pose.getY() && pose.getX() >= 0 && pose.getY() >= 0 && pose.getTranslation().getX() != 0;
    }

    public String getName(){
        return key;
    }

    public Pose2d[] getTagPoses(){
        return inputs.tagPoses;
    }
}
