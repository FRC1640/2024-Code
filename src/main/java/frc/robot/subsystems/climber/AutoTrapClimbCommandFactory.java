package frc.robot.subsystems.climber;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.drive.DriveSubsystem;
import frc.robot.Constants.TargetingConstants;
import frc.robot.sensors.Gyro.Gyro;
import frc.robot.subsystems.drive.DriveWeightCommand;
import frc.robot.subsystems.drive.DriveWeights.AutoDriveWeight;
import frc.robot.subsystems.drive.DriveWeights.ClimberAlignWeight;
import frc.robot.subsystems.drive.DriveWeights.DriveForwardRobotRelativeWeight;
import frc.robot.subsystems.drive.DriveWeights.DriveForwardRobotRelativeWeight;
import frc.robot.subsystems.drive.DriveWeights.RotateToAngleWeight;
import frc.robot.subsystems.extension.ExtensionSubsystem;
import frc.robot.subsystems.targeting.TargetingSubsystem;

public class AutoTrapClimbCommandFactory {
    private ClimberSubsystem climberSubsystem;
	private Supplier<Pose2d> currentLocation;
	private Supplier<Pose2d> getNearestStage;
	private Gyro gyro;
	private ClimberAlignWeight climberAlignWeight;
	DriveForwardRobotRelativeWeight driveForwardRobotRelativeWeight;
	private RotateToAngleWeight rotateToStageWeight;
	private DriveSubsystem driveSubsystem;
	private ExtensionSubsystem extensionSubsystem;
	private TargetingSubsystem targetingSubsystem;


    public AutoTrapClimbCommandFactory (ClimberSubsystem climberSubsystem, Supplier<Pose2d> currentLocation, Supplier<Pose2d> getNearestStage, Gyro gyro, DriveSubsystem driveSubsystem, ExtensionSubsystem extensionSubsystem, TargetingSubsystem targetingSubsystem){
		this.climberSubsystem = climberSubsystem;
		this.currentLocation = currentLocation;
		this.getNearestStage = getNearestStage;
		this.gyro = gyro;
		this.driveSubsystem = driveSubsystem;
		this.extensionSubsystem = extensionSubsystem;
		this.targetingSubsystem = targetingSubsystem;


		rotateToStageWeight = new RotateToAngleWeight(
			() -> (getNearestStage.get().getRotation().getRadians()), 
			driveSubsystem::getPose, 
			(()->0.0), 
			"rotateToStageWeight", 
			()->false, 
			this.driveSubsystem);
		
		climberAlignWeight = new ClimberAlignWeight(() -> climberSubsystem.getRightProximitySensor(), () -> climberSubsystem.getLeftProximitySensor(), gyro::getAngleRotation2d);

		driveForwardRobotRelativeWeight = new DriveForwardRobotRelativeWeight(0.5,0.1 ,gyro::getAngleRotation2d);
		
    }

    public Command getCompleteCommand(){
	 	Command c = new SequentialCommandGroup(
			new ParallelCommandGroup(getRotateToStageCommand(), lowerExtensionAndTargettingForUnderChainCommand()), 
			getBackupToStageCommand(), 
			raiseClimbersAndTargettingToClimbCommand(), 
			driveToChainWeightCommand(),
			setExtensionTargettingClimbersCommand(110,105,-5));
	 	return c;
	}

	public Command getRotateToStageCommand(){
		return new InstantCommand(()->DriveWeightCommand.addWeight(rotateToStageWeight))
					.andThen(new WaitUntilCommand(() -> proximityCancelCondition()))
					.finallyDo(()->DriveWeightCommand.removeWeight(rotateToStageWeight));
	}

	
	public Command setExtensionTargettingClimbersCommand(double extensionPos, double targettingAngle, double climberAngles){
		return new ParallelCommandGroup(targetingSubsystem.anglePIDCommand(targettingAngle), climberSubsystem.climberPIDCommandVoltage(() -> climberAngles,() -> climberAngles), extensionSubsystem.extensionPIDCommand(extensionPos));
	}
	
	public Command lowerExtensionAndTargettingForUnderChainCommand(){
		//return new ParallelCommandGroup(targetingSubsystem.anglePIDCommand(30), climberSubsystem.climberPIDCommandVoltage(() ->0,() ->0), extensionSubsystem.extensionPIDCommand(0));
		return setExtensionTargettingClimbersCommand(0, 30, 0);
	}

	public Command getBackupToStageCommand(){
		// Command c = new Command() {
		
		Command c = new InstantCommand(()->DriveWeightCommand.addWeight(climberAlignWeight))
					.andThen(new WaitUntilCommand(() -> climberAlignWeight.cancelCondition()))
					.andThen(() ->DriveWeightCommand.removeWeight(climberAlignWeight)).finallyDo(()->DriveWeightCommand.removeWeight(climberAlignWeight));
		return c;
	}

	public Command raiseClimbersAndTargettingToClimbCommand(){
		return new ParallelCommandGroup(targetingSubsystem.anglePIDCommand(100), climberSubsystem.climberPIDCommandVoltage(() -> 85,() ->85));
	}

	public Command driveToChainWeightCommand(){
		Command c = new InstantCommand(()->DriveWeightCommand.addWeight(driveForwardRobotRelativeWeight))
					.andThen(new WaitUntilCommand(() -> driveForwardRobotRelativeWeight.cancelCondition()))
					.andThen(() ->DriveWeightCommand.removeWeight(driveForwardRobotRelativeWeight))
					.finallyDo(()->DriveWeightCommand.removeWeight(driveForwardRobotRelativeWeight));
		return c;
	}




	public Command raiseExtensionToTrapLimCommand(){
		return extensionSubsystem.extensionPIDCommand(TargetingConstants.extensionUpperLimitTrap);
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
