package frc.robot.sensors.Vision.MLVision;



import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.lib.drive.DriveSubsystem;
import frc.lib.periodic.PeriodicBase;
import frc.robot.Constants.PIDConstants;


public class MLVision extends PeriodicBase {
    private MLVisionIO io;
    private MLVisionIOInputsAutoLogged inputs = new MLVisionIOInputsAutoLogged();

    public MLVision(MLVisionIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);

        Logger.processInputs("ML Vision", inputs);

    }
    


    // Getters 

    public double getLatency(){
        return inputs.latency;
    }
    public boolean isTarget(){
        return inputs.isTarget;
    }

    public double getTX(){
        if (inputs.isTargetNote){   
            return inputs.calculatedTx;
        }
        else{
            return inputs.tx;
        }
    }
    
    public double getTY(){
        if (inputs.isTargetNote){   
            return inputs.calculatedTy;
        }
        else{
            return inputs.ty;
        }
    }
    
    public double getTA(){
        if (inputs.isTargetNote){   
            return inputs.calculatedTa;
        }
        else{
            return inputs.ta;
        }
    }
   
    }