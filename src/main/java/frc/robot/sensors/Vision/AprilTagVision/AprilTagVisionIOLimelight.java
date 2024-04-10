package frc.robot.sensors.Vision.AprilTagVision;

import java.util.Arrays;

import com.fasterxml.jackson.core.format.InputAccessor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.vision.LimelightHelpers;
import frc.robot.Constants.FieldConstants;
import pabeles.concurrency.IntObjectConsumer;

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
        double[] emptyArraySix = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

        double[] botPose = networkTable.getEntry("botpose_orb_wpiblue").getDoubleArray(emptyArray);

        double[] botPoseRot = networkTable.getEntry("botpose_wpiblue").getDoubleArray(emptyArray);

        Rotation2d aprilTagBotRotation2d1 = Rotation2d.fromDegrees(botPoseRot[5]);

        Translation2d aprilTagBotTran2dRot = new Translation2d(botPoseRot[0], botPoseRot[1]);

        inputs.aprilTagPoseRot = new Pose2d(aprilTagBotTran2dRot,aprilTagBotRotation2d1);
        

        

        inputs.latency = Timer.getFPGATimestamp() - (botPose[6] / 1000.0);
        Translation2d aprilTagBotTran2d = new Translation2d(botPose[0], botPose[1]);

        
        Rotation2d aprilTagBotRotation2d = Rotation2d.fromDegrees(botPose[5]);
        inputs.aprilTagPose = new Pose2d(aprilTagBotTran2d, aprilTagBotRotation2d);
        inputs.isTarget = networkTable.getEntry("tv").getDouble(0) > 0;

        double[] robotPoseArray = networkTable.getEntry("targetpose_robotspace").getDoubleArray(emptyArraySix);
        // System.out.println(robotPoseArray + "robotPoseArray len "+
        // robotPoseArray.length);
        Translation3d robotPoseTranslation = new Translation3d(robotPoseArray[0], robotPoseArray[1], robotPoseArray[2]);

        llresults = LimelightHelpers.getLatestResults(key);

        inputs.aprilTagDistances = Arrays.stream(llresults.targetingResults.targets_Fiducials)
                .mapToDouble((v) -> v.getRobotPose_TargetSpace2D().getTranslation().getNorm()).toArray();

        // inputs.aprilTagDistance = robotPoseTranslation.getNorm();
        inputs.aprilTagDistance = botPose[9];

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

        inputs.numVisibleTags = (int)botPose[7];

        inputs.tx = networkTable.getEntry("tx").getDouble(0);

        inputs.ta = networkTable.getEntry("ta").getDouble(0);

    }
}
