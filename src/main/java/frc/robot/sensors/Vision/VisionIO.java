package frc.robot.sensors.Vision;
import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.sensors.Gyro.GyroIO.GyroIOInputs;


public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs{
        
        public long pipeline = 0;
        //public double camMode = 0.0;
        public double ledMode = 0.0;
        public double horizontal = 0.0;
        public double vertical = 0.0;
        public double targetArea = 0.0;
        public double foundTag = 0.0;

        public boolean foundTagBool = false;
        
        // Variables for distance calculations.
        //public boolean useTrigForDistanceCalc = false;
        //public double targetAreaDistance = 0.0;
        //public double trigDistance = 0.0;
        //public double distance = 0.0;

        // Variables for Limelight trigonometry calculations.
        //public double limelightAngle = 0.0;
        //public double limelightLensHeight = 0.0;
        //public double goalHeightCentimers = 0.0;

        // Variables for camera resolution.
        public double horizontalResolution = 0.0;
        public double verticalResolution = 0.0;
    }

    public default void updateInputs(VisionIOInputs inputs){
        
    }
  
    // idk if i'm ever changing the limelight...
    public default long setPipeline(VisionIOInputs inputs) {
        return 0;
    }
        
    
    public default void setCameraMode(VisionIOInputs inputs) {
        
    }  

    public default void setResolution(VisionIOInputs inputs) {
        
    }

}
