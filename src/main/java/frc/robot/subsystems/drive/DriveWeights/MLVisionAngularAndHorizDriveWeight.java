package frc.robot.subsystems.drive.DriveWeights;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.sensors.Vision.MLVision;

public class MLVisionAngularAndHorizDriveWeight implements DriveWeight {
    
    PIDController controller = new PIDController(1, 0, 0);
    double velocity;
    MLVision vision;
    double deadband = 0.1;
    double distanceLim = 7;
    ChassisSpeeds chassisSpeedsToTurn = new ChassisSpeeds(0,0,0);


    public MLVisionAngularAndHorizDriveWeight(MLVision vision) {
        this.vision = vision;
    }

    @Override
    public ChassisSpeeds getSpeeds() {
       
        velocity = controller.calculate(vision.getTX());
        velocity = (Math.abs(velocity) < deadband) ? 0 : velocity;
        velocity = MathUtil.clamp(velocity, -1, 1);
        
        if (Math.abs(vision.getTX()) > distanceLim ){
          chassisSpeedsToTurn = new ChassisSpeeds(0, 0, velocity); }
        else {          
           chassisSpeedsToTurn = new ChassisSpeeds(velocity, 0, 0);
        }
        
        return chassisSpeedsToTurn;
    }


}
