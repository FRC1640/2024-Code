package frc.robot.sensors.Vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface AprilTagVisionIO {
    @AutoLog
    public static class AprilTagVisionIOInputs {
        public double latency;
        public Pose2d aprilTagPose;
        public boolean isTarget;
        public double aprilTagDistance;
    }
    public default void updateInputs(AprilTagVisionIOInputs inputs) {
    }
}
