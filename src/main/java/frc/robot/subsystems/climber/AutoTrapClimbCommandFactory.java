package frc.robot.subsystems.climber;

import frc.robot.sensors.Gyro.Gyro;
import frc.robot.subsystems.drive.DriveWeightCommand;
import frc.robot.subsystems.drive.DriveWeights.AutoDriveWeight;
import frc.robot.subsystems.drive.DriveWeights.ClimberAlignWeight;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.targeting.TargetingSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class AutoTrapClimbCommandFactory {
    private ClimberSubsystem climberSubsystem;
	private TargetingSubsystem targetingSubsystem;
	private IntakeSubsystem intakeSubsystem;
	private ShooterSubsystem shooterSubsystem;
	private Supplier<Pose2d> currentLocation;
	private Supplier<Pose2d> getNearestStage;
	private Gyro gyro;
	private AutoDriveWeight autoStageAlignWeight;
	private ClimberAlignWeight climberAlignWeight;
	private boolean isRedAlliance;
	private Rotation2d targetAngle;


    public AutoTrapClimbCommandFactory (ClimberSubsystem climberSubsystem,TargetingSubsystem targetingSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, Supplier<Pose2d> currentLocation, Supplier<Pose2d> getNearestStage, Gyro gyro, Supplier<Integer> speakerIndex, boolean isRedAlliance){
		this.climberSubsystem = climberSubsystem;
		this.targetingSubsystem = targetingSubsystem;
		this.intakeSubsystem = intakeSubsystem;
		this.shooterSubsystem = shooterSubsystem;
		this.currentLocation = currentLocation;
		this.getNearestStage = getNearestStage;
		this.gyro = gyro;
		this.isRedAlliance = isRedAlliance;
		autoStageAlignWeight = new AutoDriveWeight(getNearestStage, currentLocation, gyro);
		
		climberAlignWeight = new ClimberAlignWeight(climberSubsystem.getDigitalInput(0), climberSubsystem.getDigitalInput(1));;
    }

    // public Command getCompleteCommand(){
	// 	Command c = new Command();
	// 	return c;
	// }

	public Command getAlignToStageCommand(){
		// Command c = new Command() {
		
		Command c = new InstantCommand(()->DriveWeightCommand.addWeight(autoStageAlignWeight))
			.andThen(new WaitUntilCommand(() -> currentLocation.get().getTranslation().getDistance(getNearestStage.get().getTranslation()) < 0.05))
			.andThen(() -> DriveWeightCommand.removeWeight(autoStageAlignWeight));
		return c;
	}

	public Command getBackupToStageCommand(){
		// Command c = new Command() {
		
		Command c = new InstantCommand(()->DriveWeightCommand.addWeight(climberAlignWeight))
					.andThen(new WaitUntilCommand(() -> climberAlignWeight.cancelCondition()))
					.andThen(() ->DriveWeightCommand.removeWeight(climberAlignWeight));
		return c;
	}

	public Command prepareForClimbCommand() {
		Command c = new InstantCommand(() -> {
			climberSubsystem.climberPIDCommand(90,90);
			targetingSubsystem.anglePIDCommand(110);
		});
		
		return c;
	}

	public Command climbAndTrapCommand() {
		Command c = new InstantCommand(() -> {climberSubsystem.climberPIDCommand(0, 0);})
		.alongWith(shooterSubsystem.setSpeedPercentPID(()->0.03, ()->0.15,
													   ()->0.03, ()->0.15, ()->true))
		.andThen(intakeSubsystem.intakeCommand(0,1, () -> (targetingSubsystem.isAnglePositionAccurate(0.1) && shooterSubsystem.isSpeedAccurate(1))));
		
		return c;
	}

	public Command manualRotateToStageCommand(){
		// Command c = new Command() {
		if (isRedAlliance){

		}
		
		Command c = new Command(){
		
		};
			return c;

	}
	public Command cancelAutoTrap(){
		Command c = new InstantCommand(() -> {DriveWeightCommand.removeWeight(autoStageAlignWeight);
		DriveWeightCommand.removeWeight(climberAlignWeight);});
		return c;
	}

    
}
