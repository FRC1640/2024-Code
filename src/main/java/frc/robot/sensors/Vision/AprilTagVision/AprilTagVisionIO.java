package frc.robot.sensors.Vision.AprilTagVision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface AprilTagVisionIO {
    @AutoLog
    public static class AprilTagVisionIOInputs {
        public double latency;
        public Pose2d aprilTagPose = new Pose2d();
        public Pose2d aprilTagPoseRot = new Pose2d();
        public boolean isTarget;
        public double[] aprilTagDistances;
        public double aprilTagDistance;
        public double ta;
        public int numVisibleTags;
        public double tx;
        // public LimelightTarget_Fiducial[] tags;

        public double tl;
        public double cl;
        public double jl;

        public Pose2d[] tagPoses;


    }
    public default void updateInputs(AprilTagVisionIOInputs inputs) {
    }
}
