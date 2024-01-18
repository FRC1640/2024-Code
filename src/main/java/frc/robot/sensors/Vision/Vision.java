package frc.robot.sensors.Vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.periodic.PeriodicBase;

public class Vision extends PeriodicBase {
    private VisionIO io;
    private VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    public Vision(VisionIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);
    }

    public Pose2d getAprilTagPose2d(){
        Translation2d aprilTagBotTran2d = new Translation2d(inputs.botPose[0], inputs.botPose[1]);
        Rotation2d aprilTagBotRotation2d = new Rotation2d(inputs.botPose[3], inputs.botPose[4]);
        return new Pose2d(aprilTagBotTran2d, aprilTagBotRotation2d);
    }
    public double getLatency(){
        double latency = Timer.getFPGATimestamp() - (inputs.botPose[5] / 1000.0);
        return latency;
    }
    public boolean isTarget(){
        return inputs.isTarget;
    }
}
