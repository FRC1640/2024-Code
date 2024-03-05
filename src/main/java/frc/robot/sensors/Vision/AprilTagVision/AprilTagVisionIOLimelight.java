package frc.robot.sensors.Vision.AprilTagVision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.vision.LimelightHelpers;

public class AprilTagVisionIOLimelight implements AprilTagVisionIO {
    String key;
    
    LimelightHelpers.LimelightResults  llresults;
    LimelightHelpers.LimelightTarget_Fiducial[] resultsArray; // FIDUCIAL IS APRIL TAGS

    public AprilTagVisionIOLimelight(String key){
        this.key = key;
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        NetworkTable networkTable = NetworkTableInstance.getDefault().getTable(key);
        double[] emptyArray = { 0, 0, 0, 0, 0, 0, 0 };
        double[] emptyArraySix = { 0, 0, 0, 0, 0, 0 };

        double[] botPose = networkTable.getEntry("botpose_wpiblue").getDoubleArray(emptyArray);
        inputs.latency = Timer.getFPGATimestamp() - (botPose[6] / 1000.0);
        Translation2d aprilTagBotTran2d = new Translation2d(botPose[0], botPose[1]);
        Rotation2d aprilTagBotRotation2d = Rotation2d.fromDegrees(botPose[5]);
        inputs.aprilTagPose = new Pose2d(aprilTagBotTran2d, aprilTagBotRotation2d);
        inputs.isTarget = networkTable.getEntry("tv").getDouble(0) > 0;
       
        double[] robotPoseArray = networkTable.getEntry("targetpose_robotspace").getDoubleArray(emptyArraySix);
        // System.out.println(robotPoseArray + "robotPoseArray len "+ robotPoseArray.length);
        Translation3d robotPoseTranslation = new Translation3d(robotPoseArray[0], robotPoseArray[1], robotPoseArray[2]);
        inputs.aprilTagDistance = robotPoseTranslation.getNorm();

        llresults = LimelightHelpers.getLatestResults(key);

        resultsArray = llresults.targetingResults.targets_Fiducials;

        inputs.numVisibleTags = (inputs.isTarget) ? resultsArray.length : 0;

        inputs.ta = networkTable.getEntry("ta").getDouble(0);


    }
}
