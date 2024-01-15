package frc.robot.sensors.Vision;



import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.periodic.PeriodicBase;
import frc.robot.Constants;


public class Vision extends PeriodicBase {
    private VisionIO io;
    private VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    
    private Translation2d botTran2d;
    private Rotation2d botRotation2d;
    private Pose2d botPose2d;
    private double latency;


    public Vision(VisionIO io){
        this.io = io;

        botTran2d = new Translation2d(inputs.botPose[0], inputs.botPose[1]);
        botRotation2d = new Rotation2d(inputs.botPose[3], inputs.botPose[4]);
        botPose2d = new Pose2d(botTran2d, botRotation2d);

        latency = Timer.getFPGATimestamp() - (inputs.botPose[6]/1000.0);

    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);
    }

   //pseudocode for the "latency" component of WPILib' addVisionMeasurement(): Timer.getFPGATimestamp() - (tl/1000.0) - (cl/1000.0) or Timer.getFPGATimestamp() - (botpose[6]/1000.0) 

   public void addVisionMeasurement (SwerveDrivePoseEstimator poseEstimator){
    poseEstimator.addVisionMeasurement(botPose2d, latency);
   }

   
    
}
