package frc.robot.subsystems.drive.DriveWeights;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.sensors.Vision.MLVision;

public class MLVisionRotationDriveWeight implements DriveWeight {
    
    PIDController angularController = new PIDController(1, 0, 0);
    double angularVelocity;
    MLVision vision;
    double deadband = 0.1;


    public MLVisionRotationDriveWeight(MLVision vision) {
        this.vision = vision;
    }

    @Override
    public ChassisSpeeds getSpeeds() {
        angularVelocity = angularController.calculate(vision.getTX());
        angularVelocity = (Math.abs(angularVelocity) < deadband) ? 0 : angularVelocity;
        angularVelocity = MathUtil.clamp(angularVelocity, -1, 1);

        ChassisSpeeds chassisSpeedsToTurn = new ChassisSpeeds(0, 0, angularVelocity);

        return chassisSpeedsToTurn;
    }


}
