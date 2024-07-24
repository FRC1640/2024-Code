package frc.robot.sensors.Vision.MLVision;

import edu.wpi.first.math.geometry.Translation2d;

public class Note {
    public Translation2d pose;
    public double confidence;

    public Note(Translation2d pose, double confidence){
        this.pose = pose;
        this.confidence = confidence;

    }
}
