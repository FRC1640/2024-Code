package frc.lib.drive.Module;

import frc.robot.Constants.PivotId;
//class to store constants for a module
public class ModuleInfo{
    
    PivotId id;
    int driveChannel;
    int steerChannel;
    int resolverChannel;
    double angleOffset;
    boolean reverseDrive;
    boolean reverseSteer;
    boolean reverseAngle;
    public ModuleInfo(PivotId id,
        int driveChannel,
        int steerChannel,
        int resolverChannel,
        double angleOffset,
        boolean reverseDrive,
        boolean reverseSteer,
        boolean reverseAngle){
            this.driveChannel = driveChannel;
            this.steerChannel = steerChannel;
            this.resolverChannel = resolverChannel;
            this.angleOffset = angleOffset;
            this.reverseDrive = reverseDrive;
            this.reverseSteer = reverseSteer;
            this.reverseAngle = reverseAngle;
        }
}