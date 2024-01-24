package frc.robot.subsystems.drive.DriveWeights;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.sensors.Vision.MLVision;

public class MLVisionHorizontalCenteringDriveWeight implements DriveWeight {
    Supplier<Double> tx;
    PIDController horizontalPIDController = new PIDController(1, 0, 0);
    double horizontalVelocity;
    MLVision vision;
    double deadband;


    public MLVisionHorizontalCenteringDriveWeight(MLVision vision) {
        this.vision = vision;
    }

    @Override
    public ChassisSpeeds getSpeeds() {
        horizontalVelocity = horizontalPIDController.calculate(vision.getTX());
        horizontalVelocity = (Math.abs(horizontalVelocity) < deadband) ? 0 : horizontalVelocity;
        horizontalVelocity = MathUtil.clamp(horizontalVelocity, -1, 1);

        ChassisSpeeds chassisSpeedsToTurn = new ChassisSpeeds(horizontalVelocity, 0, 0);

        return chassisSpeedsToTurn;
    }


}