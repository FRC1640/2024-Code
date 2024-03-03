package frc.robot.sensors.Vision.AprilTagVision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface AprilTagVisionIO {
    @AutoLog
    public static class AprilTagVisionIOInputs {
        public double latency;
        public Pose2d aprilTagPose = new Pose2d();
        public boolean isTarget;
        public double aprilTagDistance;

        public int numVisibleTags;
        

    }
    public default void updateInputs(AprilTagVisionIOInputs inputs) {
    }
}
