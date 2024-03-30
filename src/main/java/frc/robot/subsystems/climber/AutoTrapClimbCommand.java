package frc.robot.subsystems.climber;

import frc.robot.sensors.Gyro.Gyro;
import frc.robot.subsystems.drive.DriveWeightCommand;
import frc.robot.subsystems.drive.DriveWeightCommand;
import frc.robot.subsystems.drive.DriveWeights.AutoDriveWeight;
import frc.robot.subsystems.drive.DriveWeights.ClimberAlignWeight;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoTrapClimbCommand {
    private ClimberSubsystem climberSubsystem;
	private Supplier<Pose2d> currentLocation;
	private Supplier<Pose2d> getNearestStage;
	private Gyro gyro;


    public AutoTrapClimbCommand (ClimberSubsystem climberSubsystem, Supplier<Pose2d> currentLocation, Supplier<Pose2d> getNearestStage, Gyro gyro){
		climberSubsystem = this.climberSubsystem;
		currentLocation = this.currentLocation;
		getNearestStage = this.getNearestStage;
		gyro = this.gyro;
    }

    public Command getCommand(){
		Command c = new Command() {
			AutoDriveWeight autoStageAlign = new AutoDriveWeight(getNearestStage, currentLocation, gyro);
			ClimberAlignWeight finalAlign = new ClimberAlignWeight(climberSubsystem.getDigitalInput(0), climberSubsystem.getDigitalInput(1));
			
			@Override
			public void initialize(){
				DriveWeightCommand.addWeight(autoStageAlign);
			}

			@Override
			public void execute(){
				if(Math.sqrt(Math.pow(currentLocation.get().minus(getNearestStage.get()).getX(),2) + Math.pow(currentLocation.get().minus(getNearestStage.get()).getY(),2))<0.05){
					DriveWeightCommand.removeWeight(autoStageAlign);
					DriveWeightCommand.addWeight(finalAlign);
				}
			}
		};		
		return c;
	}
    
}
