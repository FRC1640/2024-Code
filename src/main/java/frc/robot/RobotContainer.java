// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Gram;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.core.sym.Name;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.drive.DriveSubsystem;
import frc.lib.drive.DriveToPosAndRotate;
import frc.lib.drive.DriveToPosAndRotateCurve;
import frc.lib.drive.DriveWeightCommand;
import frc.lib.drive.SwerveAlgorithms;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.SwerveDriveDimensions;
import frc.robot.Constants.TargetingConstants;
import frc.robot.sensors.Gyro.Gyro;
import frc.robot.sensors.Gyro.GyroIO;
import frc.robot.sensors.Gyro.GyroIONavX;
import frc.robot.sensors.Gyro.GyroIOSim;
import frc.robot.sensors.Vision.AprilTagVision.AprilTagVision;
import frc.robot.sensors.Vision.AprilTagVision.AprilTagVisionIO;
import frc.robot.sensors.Vision.AprilTagVision.AprilTagVisionIOLimelight;
import frc.robot.sensors.Vision.AprilTagVision.AprilTagVisionIOSim;
import frc.robot.sensors.Vision.MLVision.DriveToNoteCommand;
import frc.robot.sensors.Vision.MLVision.MLVision;
import frc.robot.sensors.Vision.MLVision.MLVisionIO;
import frc.robot.sensors.Vision.MLVision.MLVisionIOLimelight;
import frc.robot.sensors.Vision.MLVision.MLVisionIOSim;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOSparkMax;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveWeights.AutoDriveWeight;
import frc.robot.subsystems.drive.DriveWeights.JoystickDriveWeight;
import frc.robot.subsystems.drive.DriveWeights.MLVisionWeight;
import frc.robot.subsystems.drive.DriveWeights.RotateLockWeight;
import frc.robot.subsystems.drive.DriveWeights.RotateToAngleWeight;
import frc.robot.subsystems.extension.ExtensionIO;
import frc.robot.subsystems.extension.ExtensionIOSim;
import frc.robot.subsystems.extension.ExtensionIOSparkMax;
import frc.robot.subsystems.extension.ExtensionSubsystem;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.targeting.TargetingIO;
import frc.robot.subsystems.targeting.TargetingIOSim;
import frc.robot.subsystems.targeting.TargetingIOSparkMax;
import frc.robot.subsystems.targeting.TargetingSubsystem;
import frc.robot.util.dashboard.PIDUpdate;
import frc.robot.util.drive.MovingWhileShooting;

public class RobotContainer {

	private Gyro gyro;
	private AprilTagVision aprilTagVision1;
	private AprilTagVision aprilTagVision2;
	private MLVision mlVision;

	private DriveSubsystem driveSubsystem;
	private final CommandXboxController driveController = new CommandXboxController(0);
	private final CommandXboxController operatorController = new CommandXboxController(1);
	private final XboxController driveControllerHID = driveController.getHID();
	private final XboxController operatorControllerHID = operatorController.getHID();
	private ShooterSubsystem shooterSubsystem;
	private IntakeSubsystem intakeSubsystem;
	private ClimberSubsystem climberSubsystem;
	private TargetingSubsystem targetingSubsystem;
	private ExtensionSubsystem extensionSubsystem;

	double angleOffset = 0;

	RotateLockWeight rotateLockWeight;

	// RotateToAngleWeight rotateLockWeight;

	AutoDriveWeight autoDriveWeight;

	//MLVisionAngularAndHorizDriveWeight mlVisionWeight;
	MLVisionWeight mlVisionWeight;

	JoystickDriveWeight joystickDriveWeight;

	MovingWhileShooting movingWhileShooting;

	RotateToAngleWeight movingWhileShootingWeight;
	
	RotateToAngleWeight rotateToStageWeight;

	boolean autoTargetBool = false;

	private boolean startAuto = false;

	double autoSetAngle;

	boolean canShoot = true;
	double angleRotate = 0;

	public double autoSpeeds = 0.6;
	

	public RobotContainer() {
		ArrayList<AprilTagVision> visions = new ArrayList<>();
		switch (Robot.getMode()) {
			case REAL:
				extensionSubsystem = new ExtensionSubsystem(new ExtensionIOSparkMax());
				gyro = new Gyro(new GyroIONavX());
				aprilTagVision1 = new AprilTagVision(new AprilTagVisionIOLimelight("limelight-front"), "-front");
				aprilTagVision2 = new AprilTagVision(new AprilTagVisionIOLimelight("limelight-back"), "-back");
				visions.add(aprilTagVision1);
				visions.add(aprilTagVision2);
				driveSubsystem = new DriveSubsystem(gyro, visions);
			 	mlVision = new MLVision(new MLVisionIOLimelight(),()->driveSubsystem.getPose());

				shooterSubsystem = new ShooterSubsystem(new ShooterIOSparkMax());
				// shooterSubsystem = new ShooterSubsystem(new ShooterIO(){});
				intakeSubsystem = new IntakeSubsystem(
						new IntakeIOSparkMax(() -> driveControllerHID.getPOV() == 0, ()->intakeSubsystem.isShooting(), ()->startAuto));
				climberSubsystem = new ClimberSubsystem(new ClimberIOSparkMax());
				// intakeSubsystem = new IntakeSubsystem(new IntakeIO(){});
				targetingSubsystem = new TargetingSubsystem(new TargetingIOSparkMax(), ()->angleOffset, ()->getAlliance() == Alliance.Blue);
				// targetingSubsystem = new TargetingSubsystem(new TargetingIO() {});
				break;
			case SIM:
				extensionSubsystem = new ExtensionSubsystem(new ExtensionIOSim());
				gyro = new Gyro(new GyroIOSim(() -> Math.toDegrees(SwerveDriveDimensions.kinematics
						.toChassisSpeeds(
							driveSubsystem.getActualSwerveStates()).omegaRadiansPerSecond)));
				shooterSubsystem = new ShooterSubsystem(new ShooterIOSim());
				aprilTagVision1 = new AprilTagVision(new AprilTagVisionIOSim("limelight-front"),"-front");
				aprilTagVision2 = new AprilTagVision(new AprilTagVisionIOSim("limelight-back"),"-back");
				visions.add(aprilTagVision1);
				visions.add(aprilTagVision2);
				driveSubsystem = new DriveSubsystem(gyro, visions);
				mlVision = new MLVision(new MLVisionIOSim(),()->driveSubsystem.getPose());

				intakeSubsystem = new IntakeSubsystem(new IntakeIOSim(() -> driveControllerHID.getPOV() == 0));
				climberSubsystem = new ClimberSubsystem(new ClimberIOSim());
				targetingSubsystem = new TargetingSubsystem(new TargetingIOSim(), ()->angleOffset, ()->getAlliance() == Alliance.Blue);
				break;

			default:
				extensionSubsystem = new ExtensionSubsystem(new ExtensionIO(){});
				gyro = new Gyro(new GyroIO() {
				});
				shooterSubsystem = new ShooterSubsystem(new ShooterIO() {
				});
				aprilTagVision1 = new AprilTagVision(new AprilTagVisionIO() {
				},"-front");
				aprilTagVision2 = new AprilTagVision(new AprilTagVisionIO() {
				},"-back");
				visions.add(aprilTagVision1);
				visions.add(aprilTagVision2);
				driveSubsystem = new DriveSubsystem(gyro, visions);
				intakeSubsystem = new IntakeSubsystem(new IntakeIO() {
				});
				targetingSubsystem = new TargetingSubsystem(new TargetingIO() {
				},()->angleOffset, ()->getAlliance() == Alliance.Blue);
				climberSubsystem = new ClimberSubsystem(new ClimberIO() {
				});
				mlVision = new MLVision(new MLVisionIO() {
				}, ()->driveSubsystem.getPose());
				break;
		}
		
		

		
		// shooterSubsystem.setDefaultCommand(shooterSubsystem.setSpeedCommand(0, 0, 0, 0));
		joystickDriveWeight = new JoystickDriveWeight(driveController, gyro);
		DriveWeightCommand.addPersistentWeight(joystickDriveWeight);
		driveSubsystem.setDefaultCommand(new DriveWeightCommand().create(driveSubsystem));

		intakeSubsystem.setDefaultCommand(intakeSubsystem.intakeNoteCommand(0.8, 0.5, () -> (intakeSubsystem.hasNote() || extensionSubsystem.getExtensionPosition() > 6)));
		// intakeSubsystem.setDefaultCommand(intakeSubsystem.intakeCommand(0, 0));


		// targetingSubsystem.setDefaultCommand(targetingSubsystem.extendAndAngleSpeed(0, 0));
		
		// configure weights

		movingWhileShooting = new MovingWhileShooting(gyro, ()->getSpeakerPos(), driveSubsystem::getPose, 
		driveSubsystem::getChassisSpeeds, targetingSubsystem);
		// targetingSubsystem.setDefaultCommand(targetingSubsystem.anglePIDCommand(()->60));
		targetingSubsystem.setDefaultCommand(autoTargetMovingWhileShooting());

		// climberSubsystem.setDefaultCommand(climberSubsystem.climberPIDCommandVoltage(()->60, ()->20));

		// targetingSubsystem.setDefaultCommand(targetingSubsystem.anglePIDCommand(()->50));
		// targetingSubsystem.setDefaultCommand(
			// targetingSubsystem.anglePIDCommand(()->movingWhileShooting.getNewTargetingAngle()));
		// shooterSubsystem.setDefaultCommand(shooterSubsystem.setSpeedPercentPID(()->0.05, ()->0.1, ()->0.05, ()->0.1, ()->get3dDistance(()->getSpeakerPos()) < 999999));
		shooterSubsystem.setDefaultCommand(shooterSubsystem.setSpeedPercentPID(()->0.7, ()->0.7, ()->0.6, ()->0.6, ()->autoTargetBool));
		// shooterSubsystem.setDefaultCommand(
		// 	shooterSubsystem.setSpeedCommand(movingWhileShooting.speedToPercentOutput()));


		
		
		generateNamedCommands();



		rotateLockWeight = new RotateLockWeight(
				() -> (getAlliance() == Alliance.Blue
						? new Pose2d(FieldConstants.speakerPositionBlue, new Rotation2d())
						: new Pose2d(FieldConstants.speakerPositionRed, new Rotation2d())),
				driveSubsystem::getPose, gyro, ()->0.0);//()->aprilTagVision1.getTx(), ()->aprilTagVision1.isTarget()

		movingWhileShootingWeight = new RotateToAngleWeight(()->(get3dDistance(() -> getSpeakerPos())< FieldConstants.fullCourtShootingRadius) 
			? (movingWhileShooting.getNewRobotAngle()) 
			: (getAngleToStash()),
		driveSubsystem::getPose, ()->0.0,
			"MovingWhileShooting", ()->false, driveSubsystem);

		autoDriveWeight = new AutoDriveWeight(
				() -> (getAlliance() == Alliance.Blue
						? new Pose2d(FieldConstants.ampPositionBlue, new Rotation2d(Math.PI / 2))
						: new Pose2d(FieldConstants.ampPositionRed, new Rotation2d(Math.PI / 2))),
				driveSubsystem::getPose, gyro);

		mlVisionWeight = new MLVisionWeight(driveSubsystem, mlVision, gyro, intakeSubsystem); //new MLVisionAngularAndHorizDriveWeight(mlVision, gyro::getAngleRotation2d, ()->intakeSubsystem.hasNote());

		rotateToStageWeight = new RotateToAngleWeight(
			() -> (getNearestStage().getRotation().getRadians()), 
			driveSubsystem::getPose, 
			(()->0.0), 
			"rotateToStageWeight", 
			()->false, 
			driveSubsystem);

		DashboardInit.init(driveSubsystem, driveController, visions, targetingSubsystem, shooterSubsystem, mlVision);
		configureBindings();


	}

	private void configureBindings() {

		// NOTE: if uncommenting commented out lines here,
		// DON'T use trigger methods in CommandXboxController.
		// Use triggers & HIDs to avoid >] COMMAND OVERRUN [<.

		// driveController.leftBumper().whileTrue(intakeSubsystem.intakeCommand(0, 0.8));//.alongWith(autoTarget())

		// driveController.y().whileTrue(new RunCommand(()->driveSubsystem.rotatePivots(()->angleRotate))
		// 	.until(Math.abs(driveSubsystem.getActualSwerveStates()[0].angle.getDegrees() - angleRotate) < 3).andThen(new InstantCommand(()->{angleRotate += 90;})));
		new Trigger(() -> operatorControllerHID.getXButton())
				.whileTrue(ampCommand()); // amp shot
		new Trigger(() -> driveControllerHID.getStartButton())
				.onTrue(driveSubsystem.resetGyroCommand());
		new Trigger(() -> driveControllerHID.getBackButton())
				.onTrue(driveSubsystem.resetOdometryAprilTag());
		new Trigger(() -> driveControllerHID.getLeftBumper())
				.whileTrue(generateIntakeCommand());


		// new Trigger(()->climberSubsystem.getLeftAngle() > 5 || climberSubsystem.getRightAngle() > 5)
		// 	.onTrue(new InstantCommand(()->{autoTargetBool = false;}));
		
		// new Trigger(() -> driveControllerHID.getYButton()).whileTrue(manualShotNoAngle(35, () -> (!driveControllerHID.getYButton())));
		
		new Trigger(() -> driveControllerHID.getYButton()).onTrue(manualShot(FieldConstants.NotePresetTargetAngle, FieldConstants.NotePresetRotation[1], FieldConstants.NotePresetRotation[0], () -> (!driveControllerHID.getYButton())));
 
		// new Trigger(()->operatorControllerHID.getAButton())
		// 	.whileTrue(shooterSubsystem.setSpeedPercentPID(()->0.05, ()->0.1, ()->0.05, ()->0.1, ()->true)
		// 		.alongWith(generateIntakeCommandTrap()));
		
		// new Trigger(()->driveControllerHID.getYButton())
		// 	.whileTrue(driveSubsystem.rotateToAngleCommand(()->Math.PI/2));

		new Trigger(()->get3dDistance(()->getSpeakerPos()) > FieldConstants.fullCourtShootingRadius && Robot.inTeleop)
			.whileTrue(shooterSubsystem.setSpeedPercentPID(()->0.5, ()->0.5, ()->0.4, ()->0.4, ()->autoTargetBool));

		// static robot rotation
		// driveController.a().onTrue(new InstantCommand(() -> DriveWeightCommand.addWeight(rotateLockWeight))
		// 		.andThen(new InstantCommand(() -> joystickDriveWeight.setWeight(0.5))));
		// 		// .alongWith(autoTarget()
		// ));
		// driveController.a().onFalse(new InstantCommand(() -> DriveWeightCommand.removeWeight(rotateLockWeight))
		// 		.andThen(new InstantCommand(() -> joystickDriveWeight.setWeight(1))));
		// 		// .alongWith(Commands.race(autoTarget(), new WaitCommand(ShooterConstants.waitTime))));

		// moving while shooting robot rotation

		new Trigger(() -> driveControllerHID.getAButton())
				.onTrue(new InstantCommand(() -> DriveWeightCommand.addWeight(movingWhileShootingWeight))
				.andThen(new InstantCommand(() -> joystickDriveWeight.setWeight(1)))); // .alongWith(autoTarget()));
		new Trigger(() -> driveControllerHID.getAButton())
				.onFalse(new InstantCommand(() -> DriveWeightCommand.removeWeight(movingWhileShootingWeight))
				.andThen(new InstantCommand(() -> joystickDriveWeight.setWeight(1))));
		new Trigger(() -> operatorControllerHID.getLeftTriggerAxis() > 0.1)
				.whileTrue(targetingSubsystem.setAnglePercentOutputCommand(-0.1));
		new Trigger(() -> operatorControllerHID.getRightTriggerAxis() > 0.1)
				.whileTrue(targetingSubsystem.setAnglePercentOutputCommand(0.1));
		new Trigger(() -> driveControllerHID.getPOV() == 180)
				.onTrue(new InstantCommand(() -> toggleAutoTarget(true)));
		new Trigger(() -> operatorControllerHID.getPOV() == 180)
				.onTrue(new InstantCommand(() -> { angleOffset -= 0.5; }));
		new Trigger(() -> operatorControllerHID.getPOV() == 0)
				.onTrue(new InstantCommand(() -> { angleOffset += 0.5; }));
		new Trigger(() -> Math.abs(operatorControllerHID.getLeftY()) > 0.1 ||
				Math.abs(operatorControllerHID.getRightY()) > 0.1)
				.whileTrue(climberSubsystem.setSpeedCommand(
						() -> -operatorControllerHID.getLeftY(), () -> -operatorControllerHID.getRightY(), 
						()->SwerveAlgorithms.angleDistance(driveSubsystem.getPose().getRotation().getRadians(), Math.PI) < Math.PI / 2));
		new Trigger(() -> intakeSubsystem.hasNote())
				.onTrue(new InstantCommand(
						() -> driveControllerHID.setRumble(RumbleType.kBothRumble, 1)));
		new Trigger(() -> intakeSubsystem.hasNote())
				.onFalse(new InstantCommand(
						() -> driveControllerHID.setRumble(RumbleType.kBothRumble, 0.0)));

		
		// driveController.rightTrigger().whileTrue(new MLVisionAutoCommand2(()->intakeSubsystem.hasNote(), mlVision, driveSubsystem,()->gyro.getAngleRotation2d()).getCommand())

		new Trigger(() -> driveControllerHID.getXButton())
				.whileTrue(manualShotNoAngle(
					50,	() -> !driveControllerHID.getXButton(), true));

		// new Trigger(()->driveControllerHID.getXButton()).whileTrue(driveSubsystem.driveOnePivot(0));
		new Trigger(() -> driveControllerHID.getRightTriggerAxis() > 0.1)
				.onTrue(new InstantCommand(() -> DriveWeightCommand.addWeight(mlVisionWeight)));
		new Trigger(() -> driveControllerHID.getRightTriggerAxis() > 0.1)
		 		.onFalse(Commands.parallel(
					new InstantCommand(() -> DriveWeightCommand.removeWeight(mlVisionWeight))));
		new Trigger(() -> operatorControllerHID.getRightBumper())
				.whileTrue(
					extensionSubsystem.setExtensionPercentOutputCommand(TargetingConstants.extensionManualSpeed));
		new Trigger(() -> operatorControllerHID.getLeftBumper())
				.whileTrue(
					extensionSubsystem.setExtensionPercentOutputCommand(-TargetingConstants.extensionManualSpeed));

		new Trigger(()->driveControllerHID.getPOV() == 0).whileTrue(resetDriveCommand());

		// operatorController.rightBumper().whileTrue(targetingSubsystem.anglePIDCommand(30));
		// operatorController.leftBumper().whileTrue(targetingSubsystem.anglePIDCommand(30));


		//  new Trigger(() -> driveControllerHID.getBButton())
		// 	.whileTrue(climbCommandFactory.getCompleteCommand());

		// new Trigger (() -> driveControllerHID.getYButton()).whileTrue(climbCommandFactory.getAlignCommand());

		// new Trigger(()->operatorControllerHID.getYButton()).whileTrue(
		// 	climbCommandFactory.setExtensionTargetingClimbersCommand(5, 90, 55)
		// 	.andThen(climbCommandFactory.setExtensionTargetingClimbersCommand(100, 90, 55))
		// 	.andThen(climbCommandFactory.setExtensionTargetingClimbersCommand(100, 100, 55))
		// 	.andThen(climbCommandFactory.setExtensionTargetingClimbersCommand(150, 100, 55)));
		
		// new Trigger(() -> driveControllerHID.getBButton())
		// 	.onTrue(new InstantCommand(() -> DriveWeightCommand.addWeight(rotateToStageWeight)));

		// new Trigger(() -> driveControllerHID.getBButton())
		// 	.onFalse(new InstantCommand(() -> DriveWeightCommand.removeWeight(rotateToStageWeight)));

			// new Trigger(() -> driveControllerHID.getBButton())
		
		
		new Trigger(() -> driveControllerHID.getBButton())
			.whileTrue(manualShotNoAngle(55, () -> !driveControllerHID.getBButton()));


		// new Trigger(()->operatorControllerHID.getYButton())
		// 	.whileTrue(climberSubsystem.climberPIDCommandVoltage(()->80, ()->80).alongWith(targetingSubsystem.anglePIDCommand(()->70, 70, ()->true).repeatedly()));

		// driveController.y().whileTrue(intakeSubfsystem.intakeCommand(-0.5, 0));

		// driveController.y().onTrue(driveSubsystem.resetOdometryAprilTag());

		// new Trigger (() -> operatorControllerHID.getAButton())
		// 		.whileTrue(manualShotNoAngle(41.8, () -> !operatorControllerHID.getAButton()));

		// driveController.y().whileTrue(() -> );

		// new Trigger(()-> (operatorControllerHID.getAButton() && mlVision.getClosestNote().isPresent()))
		// 	.onTrue(new InstantCommand(()->
		// 		driveSubsystem.autoGeneratedPath(()->
		// 			(mlVision.getClosestNote().isPresent() ? mlVision.getClosestNote().get() : new Translation2d(10,10))).repeatedly()
		// 			.until(()->!(operatorControllerHID.getAButton() && mlVision.getClosestNote().isPresent())).schedule()));

		// new Trigger(() -> operatorControllerHID.getAButton() && mlVision.isTarget() && !intakeSubsystem.hasNote())
		// 	.onTrue(new ProxyCommand(()->driveSubsystem.autoGeneratedPath(()->mlVision.getClosestNote()).until(()->!operatorControllerHID.getAButton() || intakeSubsystem.hasNote())));

		// new Trigger(()->operatorControllerHID.getAButton()).whileTrue(new DriveToPosAndRotate(driveSubsystem, mlVision, gyro));
		// new Trigger(()->operatorControllerHID.getAButton()).whileTrue(DriveWeightCommand.addWeight();)
		new Trigger(()->(driveControllerHID.getPOV() == 90))
		.onTrue(new InstantCommand(()->driveSubsystem.resetPivots(), driveSubsystem));
	}

	public void pidTriggers(){
		// climberSubsystem.setDefaultCommand(climberSubsystem.climberPIDCommandVoltage(() -> PIDUpdate.getSetpoint(), () -> 0.0));
		//extensionSubsystem.setDefaultCommand(extensionSubsystem.extensionPIDCommand(()->PIDUpdate.getSetpoint()));
		targetingSubsystem.setDefaultCommand(
			targetingSubsystem.anglePIDCommand(()->PIDUpdate.getSetpoint(),()->PIDUpdate.getPID() == PIDConstants.map.get("targetingPIDSmall")));
		// shooterSubsystem.setDefaultCommand(
		// 	shooterSubsystem.setSpeedPercentPID(()->0, ()->PIDUpdate.getSetpoint(), ()->0, ()->0, 
		// 	()->PIDUpdate.getPID() == PIDConstants.map.get("bottomLeftShooter"))
		// );
		
	}

	public void resetDrive(){
		driveSubsystem.resetPivots();
	}
	public Command resetDriveCommand(){
		return driveSubsystem.pidPivotsCommand().andThen(new InstantCommand(()->driveSubsystem.resetPivots()));
	}

	public double getAngleToStash(){
		return Math.atan2(getStashPos().getY() - driveSubsystem.getPose().getY(),
                getStashPos().getX() - driveSubsystem.getPose().getX());
	}

	public void toggleAutoTarget(boolean toggled){
		Logger.recordOutput("ToggledAutoTarget", autoTargetBool);
		autoTargetBool = toggled;
	}

	public Command getAutonomousCommand() {
		return DashboardInit.getAutoChooserCommand().alongWith(new InstantCommand(()->toggleAutoTarget(true)))
				.andThen(driveSubsystem.driveDoubleConeCommand(() -> new ChassisSpeeds(), () -> new Translation2d(), ()->false))
				// .andThen(driveSubsystem.resetGyroCommand())
				.alongWith(new InstantCommand(()->Logger.recordOutput("AutoRun", DashboardInit.getAutoChooserCommand().getName())))
				.alongWith(new InstantCommand(()->{startAuto = true;}));
				// .alongWith(new InstantCommand(()->{autoTargetBool = true;}));
				// .alongWith(shooterSub\[]system.setSpeedPercentPID(()->autoSpeeds, ()->autoSpeeds, ()->autoSpeeds-0.1, ()->autoSpeeds-0.1, ()->true));
	}

	public void removeAllDefaultCommands() {
		driveSubsystem.removeDefaultCommand();
		shooterSubsystem.removeDefaultCommand();
		intakeSubsystem.removeDefaultCommand();
		targetingSubsystem.removeDefaultCommand();
		climberSubsystem.removeDefaultCommand();
	}

	private Alliance getAlliance() {
		if (DriverStation.getAlliance().isPresent()) {
			return DriverStation.getAlliance().get();
		}
		return Alliance.Blue;
	}

	public Pose2d getNearestStage(){ 
		Pose2d[] allianceStagePoses = getAlliance() == Alliance.Blue?FieldConstants.blueStages:FieldConstants.redStages;
		double shortestDistance = allianceStagePoses[0].getTranslation().getDistance(driveSubsystem.getPose().getTranslation());
		Pose2d closestPose = allianceStagePoses[0];
		int index = 0;

		for (int i = 1; i < allianceStagePoses.length ; i++ ){
			if (allianceStagePoses[i].getTranslation().getDistance(driveSubsystem.getPose().getTranslation()) < shortestDistance){
		 		shortestDistance = allianceStagePoses[i].getTranslation().getDistance(driveSubsystem.getPose().getTranslation());
				closestPose = allianceStagePoses[i];
				index = i;
			}
		}

		Logger.recordOutput("Drive/nearest speaker", index);
		Logger.recordOutput("Drive/nearest speaker pos", closestPose);
		//		Logger.recordOutput("cancel condition temp POS Dif", (closestPose.getTranslation().getDistance(driveSubsystem.getPose().getTranslation())) < 9 && Math.abs((closestPose.getRotation().getRadians() - gyro.getAngleRotation2d().getRadians())) < 0.1); // angle error < 0.1

		Logger.recordOutput("cancel condition temp POS Dif", (closestPose.getTranslation().getDistance(driveSubsystem.getPose().getTranslation()))); // angle error < 0.1
		Logger.recordOutput("cancel condition temp ANGLE DIF", (Math.abs((closestPose.getRotation().getDegrees() - driveSubsystem.getPose().getRotation().getDegrees())))); // angle error < 0.1


		return closestPose;
    }

	private Command generateIntakeCommand() {
		return intakeSubsystem.intakeCommand(0, 0.8,
				() -> (shooterSubsystem.isSpeedAccurate(0.05) && targetingSubsystem.isAnglePositionAccurate(
					(get3dDistance(() -> getSpeakerPos()) < FieldConstants.fullCourtShootingRadius ? TargetingConstants.angleError:4))
						&& Math.toDegrees(Math.abs(SwerveAlgorithms.angleDistance(
								DriveWeightCommand.getAngle(), driveSubsystem.getPose().getRotation().getRadians()))) < 
								(get3dDistance(() -> getSpeakerPos()) < FieldConstants.fullCourtShootingRadius ? 4: 8)) && canShoot);
	}

    private Command generateIntakeCommandAuto() {
			return new SequentialCommandGroup(new WaitUntilCommand(() -> !intakeSubsystem.hasNote()), new WaitCommand(0.3))
				.deadlineWith(intakeSubsystem.intakeCommand(0, 0.8,
				() -> (shooterSubsystem.isSpeedAccurate(0.3) 
					&& targetingSubsystem.isAnglePositionAccurate(2))).repeatedly());
	}

	public Command intakeNote(){
		return intakeSubsystem.intakeNoteCommand(0.8, 0.5, () -> intakeSubsystem.hasNote())
			.repeatedly().until(() -> intakeSubsystem.hasNote());
	}
	public Command generateIntakeNoRobot(double time, double indexSpeed){
		return intakeSubsystem.intakeCommand(0, indexSpeed,
				() -> (shooterSubsystem.isSpeedAccurate(0.02) 
				&& targetingSubsystem.isAnglePositionAccurate(1)), time);
	}
	public Command generateIntakeNoRobotAmp(double time, double indexSpeed){
		return intakeSubsystem.intakeCommand(0, indexSpeed,
				() -> (shooterSubsystem.isSpeedAccurate(0.1) 
				&& targetingSubsystem.isAnglePositionAccurate(6) 
				&& extensionSubsystem.isExtensionAccurate()), time);
	}

	public Pose2d getSpeakerPos() {
		return (getAlliance() == Alliance.Blue
				? new Pose2d(FieldConstants.speakerPositionBlue, new Rotation2d())
				: new Pose2d(FieldConstants.speakerPositionRed, new Rotation2d()));
	}

	public Pose2d getStashPos() {
		return (getAlliance() == Alliance.Blue
				? new Pose2d(FieldConstants.stashPositionBlue, new Rotation2d())
				: new Pose2d(FieldConstants.stashPositionRed, new Rotation2d()));
	}

	public Command manualShotNoAngle(double targetAngle, BooleanSupplier cancelCondition) {
		return targetingSubsystem.anglePIDCommand(targetAngle).alongWith(generateIntakeNoRobot(0, 0.8)).until(()->cancelCondition.getAsBoolean());
	}
	public Command manualShotNoAngle(double targetAngle, BooleanSupplier cancelCondition, boolean shotSpeeds) {
		return targetingSubsystem.anglePIDCommand(targetAngle)
		.alongWith(shooterSubsystem.setSpeedPercentPID(()->0.03, ()->0.27,
			()->0.03, ()->0.27, ()->true))
		.alongWith(generateIntakeNoRobot(0, 0.8));
	}

	public Command manualShotAuto(double targetAngle) {
		return targetingSubsystem.anglePIDCommand(targetAngle).repeatedly();
	}

	public Command fullSpeakerShot(){
		Command c = manualShotAuto(55);
		return new ParallelDeadlineGroup(generateIntakeCommandAuto(), c);
	}

	public Command rotCommand(){
		SequentialCommandGroup s = new SequentialCommandGroup(driveSubsystem.rotateToAngleCommand(()->movingWhileShooting.getNewRobotAngle()), generateIntakeCommandAuto());
		Command c = targetingSubsystem.anglePIDCommand(()->determineTargetingAngle(), 60, ()->true).repeatedly();
		return new ParallelDeadlineGroup(s, c);
	}

	public Command rotCommand(double wait){
		SequentialCommandGroup s = new SequentialCommandGroup(driveSubsystem.rotateToAngleCommand(()->movingWhileShooting.getNewRobotAngle(), wait), generateIntakeCommandAuto());
		Command c = targetingSubsystem.anglePIDCommand(()->determineTargetingAngle(), 60, ()->true).repeatedly();
		return new ParallelDeadlineGroup(s, c);
	}

	public Command manualShot(double targetAngle, double robotAngleBlueRadians, double robotAngleRedRadians, BooleanSupplier cancelCondition) {
		return targetingSubsystem.anglePIDCommand(targetAngle).alongWith(new InstantCommand(() -> {
			RotateToAngleWeight weight = new RotateToAngleWeight(
					() -> (getAlliance()==Alliance.Blue?robotAngleBlueRadians:robotAngleRedRadians), 
					driveSubsystem::getPose, ()->(
						0.0), "manualShot",
					cancelCondition, driveSubsystem);
			DriveWeightCommand.addWeight(weight);
		}).alongWith(generateIntakeCommand())).until(cancelCondition);
	}

	public double get3dDistance(Supplier<Pose2d> speakerPos) {
		Logger.recordOutput("Drive/DistanceToSpeaker",
				new Translation3d(speakerPos.get().minus(driveSubsystem.getPose()).getX(),
						speakerPos.get().minus(driveSubsystem.getPose()).getY(), 2.11).getNorm());

		
		
		return new Translation3d(speakerPos.get().minus(driveSubsystem.getPose()).getX() + 
			(FieldConstants.speakerOffset * (getAlliance() == Alliance.Blue ? 1:-1)),
				speakerPos.get().minus(driveSubsystem.getPose()).getY(), 2.11).getNorm();

	}

	// public double generateAngleToStash(){
	// 	return Math.atan2(.getY() - driveSubsystem.getPose.getY(),
	// 	goalPose.get().getX() - getPose.get().getX()) - gyro.getOffset();
	// }

	public Command autoTarget(){
		return targetingSubsystem.anglePIDCommand(() -> 
			targetingSubsystem.distToAngle(()->get3dDistance(() -> getSpeakerPos())), 60, ()->autoTargetBool);

	}

	public double determineTargetingAngle(){
		if((intakeSubsystem.hasNote() && get3dDistance(() -> getSpeakerPos()) < FieldConstants.fullCourtShootingRadius) || !Robot.inTeleop){
			canShoot = true;
			return movingWhileShooting.getAngleFromDistance();
		} else if (intakeSubsystem.hasNote() && get3dDistance(() -> getSpeakerPos()) > FieldConstants.fullCourtShootingRadius && operatorControllerHID.getBButton()){
			canShoot = true;
			return 43;
		} else {
			canShoot = false;
			return 35;
		}

		// return (get3dDistance(() -> getSpeakerPos()) < FieldConstants.fullCourtShootingRadius //10.249
		// 	? movingWhileShooting.getAngleFromDistance() : 40);
	}
	public Command autoTargetMovingWhileShooting(){
		return targetingSubsystem.anglePIDCommand(() -> determineTargetingAngle(), 60, ()->autoTargetBool
			&& extensionSubsystem.getExtensionPosition() < 20);
	}
	

	public Command ampCommand(){
		return shooterSubsystem.setSpeedPercentPID(()->0.03, ()->0.15,
			()->0.03, ()->0.15, ()->true)
			.alongWith(generateIntakeNoRobotAmp(500, 0.6))
			.alongWith(targetingSubsystem.anglePIDCommand(55));
	}

	public Command trapShotCommand(){
		
		return shooterSubsystem.setSpeedPercentPID(()->0.08, ()->0.4,
			()->0.08, ()->0.4, ()->true)
			.alongWith(targetingSubsystem.anglePIDCommand(()->70))
			// .alongWith(generateIntakeCommand())
			;
	}

	public Command ampCommandNoShoot(){
		return shooterSubsystem.setSpeedPercentPID(()->0.03, ()->0.27,
			()->0.03, ()->0.27, ()->true)
			.alongWith(targetingSubsystem.anglePIDCommand(50));
	}

	public void generateNamedCommands(){
		// double offset = 4;
		NamedCommands.registerCommand("Run Indexer", generateIntakeCommandAuto());
		NamedCommands.registerCommand("Run Intake", intakeNote());
		NamedCommands.registerCommand("SpeakerShot", manualShotAuto(55));

		NamedCommands.registerCommand("EnableAprilTags", new InstantCommand(()->driveSubsystem.setAprilTagInAuto(true)));
		NamedCommands.registerCommand("DisableAprilTags", new InstantCommand(()->driveSubsystem.setAprilTagInAuto(false)));

		NamedCommands.registerCommand("RotateToSpeaker", driveSubsystem.rotateToAngleCommand(()->movingWhileShooting.getNewRobotAngle()));

		NamedCommands.registerCommand("AutoTarget", targetingSubsystem.anglePIDCommand(()->determineTargetingAngle(), 60, ()->true).repeatedly());
		NamedCommands.registerCommand("TargetNoBounce", targetingSubsystem.anglePIDCommand(()->30, 60, ()->true).repeatedly());
		NamedCommands.registerCommand("Full Speaker Shot", fullSpeakerShot());
		NamedCommands.registerCommand("RotCommand(0)", rotCommand());
		NamedCommands.registerCommand("RotCommand(.1)", rotCommand(.1));
		NamedCommands.registerCommand("RotCommand(.2)", rotCommand(.2));
		NamedCommands.registerCommand("RotCommand(.3)", rotCommand(.3));
		NamedCommands.registerCommand("RotCommand(.35)", rotCommand(.35));
		NamedCommands.registerCommand("RotCommand(.5)", rotCommand(.5));
		NamedCommands.registerCommand("RotCommand(4)", rotCommand(4));	
		NamedCommands.registerCommand("RotCommand(1)", rotCommand(1));
		// NamedCommands.registerCommand("IntakeWait", new WaitUntilCommand(()->intakeSubsystem.hasNote()).raceWith(new WaitCommand(2)));

		NamedCommands.registerCommand("StopRobot", driveSubsystem.driveDoubleConeCommand(()->new ChassisSpeeds(), ()->new Translation2d(), ()->false));

		// NamedCommands.registerCommand("MLTestDrives", driveSubsystem.pathWithMovableEndpoint("ML Test", ()->mlVision.getClosestNotePos()));

		NamedCommands.registerCommand("DriveToNote", new DriveToPosAndRotate(driveSubsystem, mlVision, gyro, intakeSubsystem));

		NamedCommands.registerCommand("NoteValid", new WaitUntilCommand(()->
			mlVision.getClosestNotePos().getDistance(driveSubsystem.getPose().getTranslation()) < 1.5 && mlVision.getConfidence() > 0.55));

		NamedCommands.registerCommand("Print", new PrintCommand("print").repeatedly());
		NamedCommands.registerCommand("AutoShootRotation", new InstantCommand(()->driveSubsystem.setRotationTargetAuto(() -> (Rotation2d.fromRadians(movingWhileShooting.getNewRobotAngle())))));
		NamedCommands.registerCommand("StandardRotation", new InstantCommand(()->driveSubsystem.setRotationTargetAuto()));
	}
}