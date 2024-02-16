package frc.robot.sensors.Vision.AprilTagVision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class AprilTagVisionIOSim implements AprilTagVisionIO {
    DoublePublisher tv;
    DoubleArrayPublisher botpose_wpiblue, targetpose_robotspace;

    public AprilTagVisionIOSim() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        botpose_wpiblue = table.getDoubleArrayTopic("botpose_wpiblue").publish();
        botpose_wpiblue.set(new double[] { 0, 0, 0, 0, 0, 0, 0 });

        targetpose_robotspace = table.getDoubleArrayTopic("targetpose_robotspace").publish();
        targetpose_robotspace.set(new double[] { 0, 0, 0, 0, 0, 0, 0 });

        tv = table.getDoubleTopic("tv").publish();
        tv.set(0);
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("limelight");
        double[] emptyArray = { 0, 0, 0, 0, 0, 0, 0 };
        double[] botPose = networkTable.getEntry("botpose_wpiblue").getDoubleArray(emptyArray);
        inputs.latency = Timer.getFPGATimestamp() - (botPose[6] / 1000.0);
        Translation2d aprilTagBotTran2d = new Translation2d(botPose[0], botPose[1]);
        Rotation2d aprilTagBotRotation2d = Rotation2d.fromDegrees(botPose[5]);
        inputs.aprilTagPose = new Pose2d(aprilTagBotTran2d, aprilTagBotRotation2d);
        inputs.isTarget = networkTable.getEntry("tv").getDouble(0) > 0;
        double[] robotPoseArray = networkTable.getEntry("targetpose_robotspace").getDoubleArray(emptyArray);
        Translation3d robotPoseTranslation = new Translation3d(robotPoseArray[0], robotPoseArray[1], robotPoseArray[2]);
        inputs.aprilTagDistance = robotPoseTranslation.getNorm();
    }
}
