package frc.robot.sensors.Vision;

import org.littletonrobotics.junction.Logger;

import frc.lib.periodic.PeriodicBase;


public class Vision extends PeriodicBase {
    private VisionIO io;

    private VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    public Vision(VisionIO io){
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);
    }


   
    
}
