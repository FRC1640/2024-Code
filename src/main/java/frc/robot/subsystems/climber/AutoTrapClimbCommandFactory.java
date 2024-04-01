package frc.robot.subsystems.climber;

import frc.robot.sensors.Gyro.Gyro;
import frc.robot.subsystems.drive.DriveWeightCommand;
import frc.robot.subsystems.drive.DriveWeights.AutoDriveWeight;
import frc.robot.subsystems.drive.DriveWeights.ClimberAlignWeight;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class AutoTrapClimbCommandFactory {
    private ClimberSubsystem climberSubsystem;
	private Supplier<Pose2d> currentLocation;
	private Supplier<Pose2d> getNearestStage;
	private Gyro gyro;
	private AutoDriveWeight autoStageAlignWeight;
	private ClimberAlignWeight climberAlignWeight;


    public AutoTrapClimbCommandFactory (ClimberSubsystem climberSubsystem, Supplier<Pose2d> currentLocation, Supplier<Pose2d> getNearestStage, Gyro gyro){
		this.climberSubsystem = climberSubsystem;
		this.currentLocation = currentLocation;
		this.getNearestStage = getNearestStage;
		this.gyro = gyro;
		autoStageAlignWeight = new AutoDriveWeight(()->getNearestStage.get(), ()->currentLocation.get(), gyro);
		
		climberAlignWeight = new ClimberAlignWeight(() -> climberSubsystem.getRightProximitySensor(), () -> climberSubsystem.getLeftProximitySensor(), gyro::getAngleRotation2d);;
    }

    // public Command getCompleteCommand(){
	// 	Command c = new Command();
	// 	return c;
	// }

	public Command getAlignToStageCommand(){
		// Command c = new Command() {
		
		Command c = new InstantCommand(()->DriveWeightCommand.addWeight(autoStageAlignWeight)).andThen(new RunCommand(() -> {System.out.println(getNearestStage.get() + " is a " + getNearestStage.get().getClass());}).repeatedly())
					.until(() -> currentLocation.get().getTranslation().getDistance(getNearestStage.get().getTranslation()) < 0.05)
					.andThen(() ->DriveWeightCommand.removeWeight(autoStageAlignWeight));
		return c;
	}

	public Command getBackupToStageCommand(){
		// Command c = new Command() {
		
		Command c = new InstantCommand(()->DriveWeightCommand.addWeight(climberAlignWeight))
					.andThen(new WaitUntilCommand(() -> climberAlignWeight.cancelCondition()))
					.andThen(() ->DriveWeightCommand.removeWeight(climberAlignWeight)).finallyDo(()->DriveWeightCommand.removeWeight(climberAlignWeight));
		return c;
	}

    
}
