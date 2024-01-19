package frc.robot.sensors.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class VisionIOLimelight implements VisionIO {

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("limelight");
        double[] emptyArray = { 0, 0, 0, 0, 0, 0 };
        double[] botPose = networkTable.getEntry("botpose_wpiblue").getDoubleArray(emptyArray);
        inputs.latency = Timer.getFPGATimestamp() - (botPose[5] / 1000.0);
        Translation2d aprilTagBotTran2d = new Translation2d(botPose[0], botPose[1]);
        Rotation2d aprilTagBotRotation2d = new Rotation2d(botPose[3], botPose[4]);
        inputs.aprilTagPose = new Pose2d(aprilTagBotTran2d, aprilTagBotRotation2d);
        inputs.isTarget = networkTable.getEntry("tv").getDouble(0) > 0;
    }
}
