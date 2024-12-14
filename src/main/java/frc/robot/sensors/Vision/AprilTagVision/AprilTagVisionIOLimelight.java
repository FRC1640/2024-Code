package frc.robot.sensors.Vision.AprilTagVision;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.lib.vision.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.Constants.FieldConstants;

public class AprilTagVisionIOLimelight implements AprilTagVisionIO {
    String key;

    LimelightHelpers.LimelightResults llresults;
    LimelightHelpers.LimelightTarget_Fiducial[] resultsArray; // FIDUCIAL IS APRIL TAGS

    public AprilTagVisionIOLimelight(String key) {
        this.key = key;
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        NetworkTable networkTable = NetworkTableInstance.getDefault().getTable(key);
        double[] emptyArray = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

        double[] botPose = networkTable.getEntry("botpose_orb_wpiblue").getDoubleArray(emptyArray);

        double[] botPoseRot = networkTable.getEntry("botpose_wpiblue").getDoubleArray(emptyArray);

        Rotation2d aprilTagBotRotation2d1 = Rotation2d.fromDegrees(botPoseRot[5]);

        Translation2d aprilTagBotTran2dRot = new Translation2d(botPoseRot[0], botPoseRot[1]);

        
        

        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(key);

        LimelightHelpers.PoseEstimate poseRot = LimelightHelpers.getBotPoseEstimate_wpiBlue(key);

        inputs.latency = Robot.isDisabled ? poseRot.timestampSeconds: limelightMeasurement.timestampSeconds;
        inputs.aprilTagPoseRot = poseRot.pose;
        Translation2d aprilTagBotTran2d = new Translation2d(botPose[0], botPose[1]);

        
        Rotation2d aprilTagBotRotation2d = Rotation2d.fromDegrees(botPose[5]);
        
        inputs.aprilTagPose = limelightMeasurement.pose;
        
        inputs.isTarget = networkTable.getEntry("tv").getDouble(0) > 0;

        llresults = LimelightHelpers.getLatestResults(key);

        inputs.aprilTagDistances = Arrays.stream(llresults.targetingResults.targets_Fiducials)
                .mapToDouble((v) -> v.getRobotPose_TargetSpace2D().getTranslation().getNorm()).toArray();

        // inputs.aprilTagDistance = robotPoseTranslation.getNorm();
        inputs.aprilTagDistance = limelightMeasurement.avgTagDist;

        inputs.tagPoses = Arrays.stream(llresults.targetingResults.targets_Fiducials)
                .map((v) -> new Pose2d(v.getRobotPose_FieldSpace2D().getX() + FieldConstants.width / 2,
                        v.getRobotPose_FieldSpace2D().getY() + FieldConstants.height / 2,
                        v.getRobotPose_FieldSpace2D().getRotation()))
                .toArray(Pose2d[]::new);

        // inputs.latency =
        // Timer.getFPGATimestamp() - (llresults.targetingResults.latency_capture
        // + llresults.targetingResults.latency_jsonParse
        // + llresults.targetingResults.latency_pipeline);

        inputs.tl = llresults.targetingResults.latency_pipeline;
        inputs.cl = llresults.targetingResults.latency_capture;
        inputs.jl = llresults.targetingResults.latency_jsonParse;

        resultsArray = llresults.targetingResults.targets_Fiducials;

        inputs.numVisibleTags = limelightMeasurement.tagCount;

        inputs.tx = networkTable.getEntry("tx").getDouble(0);

        inputs.ta = networkTable.getEntry("ta").getDouble(0);

    }
}
