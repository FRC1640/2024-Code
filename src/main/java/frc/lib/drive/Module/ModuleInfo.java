package frc.lib.drive.Module;

import edu.wpi.first.wpilibj.I2C;
import frc.robot.Constants.PivotId;
//class to store constants for a module
public class ModuleInfo{
    
    PivotId id;
    int driveChannel;
    int steerChannel;
    I2C mux;
    int resolverAddress;
    int resolverMuxChannel;
    double angleOffset;
    boolean reverseDrive;
    boolean reverseSteer;
    boolean reverseAngle;
    double wheelRadius;
    double vSlope1;
    double vSlope2;
    
    public ModuleInfo(
        PivotId id,
        int driveChannel,
        int steerChannel,
        I2C mux,
        int resolverAddress,
        int resolverMuxChannel,
        double angleOffset,
        boolean reverseDrive,
        boolean reverseSteer,
        boolean reverseAngle,
        double vSlope1,
        double vSlope2,
        double wheelRadius){
            this.driveChannel = driveChannel;
            this.steerChannel = steerChannel;
            this.mux = mux;
            this.resolverAddress = resolverAddress;
            this.resolverMuxChannel = resolverMuxChannel;
            this.angleOffset = angleOffset;
            this.reverseDrive = reverseDrive;
            this.reverseSteer = reverseSteer;
            this.reverseAngle = reverseAngle;
            this.vSlope1 = vSlope1;
            this.vSlope2 = vSlope2;
            this.wheelRadius = wheelRadius;
        }
}