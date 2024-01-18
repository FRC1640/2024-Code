package frc.robot.sensors.Vision;
import org.littletonrobotics.junction.AutoLog;


public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs{
        
        //public long pipeline = 0;
        //public double camMode = 0.0;
        //public double ledMode = 0.0;
        //public double horizontal = 0.0;
        //public double vertical = 0.0;
        //public double targetArea = 0.0;
        //public double foundTag = 0.0;

        //public boolean foundTagBool = false;
        
        // ONLY RElEVANT THING OOPSY 
        public double[] botPose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};// (x,y,z in meters, roll, pitch, yaw in degrees, and cl+tl (total latency)). 
        //public double tl = 0.0;
        //public double cl = 0.0;

    }

    public default void updateInputs(VisionIOInputs inputs){
        
    }
  
    //public default void setPipeline(VisionIOInputs inputs) {}
        
    
    //public default void setCameraMode(VisionIOInputs inputs) { }  
    
    //public default void setLEDMode(VisionIOInputs inputs) {}  


}
