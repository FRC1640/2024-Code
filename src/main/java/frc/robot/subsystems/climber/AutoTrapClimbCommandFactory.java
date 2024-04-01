package frc.robot.subsystems.climber;

import frc.lib.drive.DriveSubsystem;
import frc.robot.sensors.Gyro.Gyro;
import frc.robot.subsystems.drive.DriveWeightCommand;
import frc.robot.subsystems.drive.DriveWeights.AutoDriveWeight;
import frc.robot.subsystems.drive.DriveWeights.ClimberAlignWeight;
import frc.robot.subsystems.drive.DriveWeights.RotateToAngleWeight;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class AutoTrapClimbCommandFactory {
    private ClimberSubsystem climberSubsystem;
	private Supplier<Pose2d> currentLocation;
	private Supplier<Pose2d> getNearestStage;
	private Gyro gyro;
	private AutoDriveWeight autoStageAlignWeight;
	private ClimberAlignWeight climberAlignWeight;
	private RotateToAngleWeight rotateToStageWeight;
	private DriveSubsystem driveSubsystem;


    public AutoTrapClimbCommandFactory (ClimberSubsystem climberSubsystem, Supplier<Pose2d> currentLocation, Supplier<Pose2d> getNearestStage, Gyro gyro, DriveSubsystem driveSubsystem){
		this.climberSubsystem = climberSubsystem;
		this.currentLocation = currentLocation;
		this.getNearestStage = getNearestStage;
		this.gyro = gyro;
		this.driveSubsystem = driveSubsystem;

		rotateToStageWeight = new RotateToAngleWeight(
			() -> (getNearestStage.get().getRotation().getRadians()), 
			driveSubsystem::getPose, 
			(()->0.0), 
			"rotateToStageWeight", 
			()->false, 
			this.driveSubsystem);
		
		climberAlignWeight = new ClimberAlignWeight(() -> climberSubsystem.getRightProximitySensor(), () -> climberSubsystem.getLeftProximitySensor(), gyro::getAngleRotation2d);;
		
    }

     public Command getCompleteCommand(){
	 	Command c = new SequentialCommandGroup(getRotateToStageCommand(), getBackupToStageCommand() );
	 	return c;
	 }

	public Command getRotateToStageCommand(){
		Command c = new InstantCommand(()->DriveWeightCommand.addWeight(rotateToStageWeight))
					.andThen(new WaitUntilCommand(() -> proximityCancelCondition()))
					.finallyDo(()->DriveWeightCommand.removeWeight(rotateToStageWeight));
		return c;
	}

	public Command getBackupToStageCommand(){
		// Command c = new Command() {
		
		Command c = new InstantCommand(()->DriveWeightCommand.addWeight(climberAlignWeight))
					.andThen(new WaitUntilCommand(() -> climberAlignWeight.cancelCondition()))
					.andThen(() ->DriveWeightCommand.removeWeight(climberAlignWeight)).finallyDo(()->DriveWeightCommand.removeWeight(climberAlignWeight));
		return c;
	}

	private boolean proximityCancelCondition(){
		return (getNearestStage.get().getTranslation().getDistance(driveSubsystem.getPose().getTranslation())) < 1 && Math.abs((getNearestStage.get().getRotation().getDegrees() - driveSubsystem.getPose().getRotation().getDegrees())) < 0.3; // angle error < 0.1
	}

	// SET Extention lim in extension command
// set soft lim extenstion HERE
// targetting ends at 105
// shooter 30
// add targetting - 30 and climber arms -5 down 

//COMING FORWAR -shooter to 90 arms 85
//BRINGING SOWN 
// shooter 100
// extenstion new soft lim
// as climber down extension and up cl dowb to 0
// final shooter 105
// final climb -5

    
}
