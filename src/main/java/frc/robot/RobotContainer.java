// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.drive.DriveSubsystem;
import frc.lib.swerve.SwerveAlgorithms;
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
import frc.robot.sensors.Vision.MLVision.MLVision;
import frc.robot.sensors.Vision.MLVision.MLVisionAutoCommand2;
import frc.robot.sensors.Vision.MLVision.MLVisionIO;
import frc.robot.sensors.Vision.MLVision.MLVisionIOLimelight;
import frc.robot.sensors.Vision.MLVision.MLVisionIOSim;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOSparkMax;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveWeightCommand;
import frc.robot.subsystems.drive.DriveWeights.AutoDriveWeight;
import frc.robot.subsystems.drive.DriveWeights.JoystickDriveWeight;
import frc.robot.subsystems.drive.DriveWeights.MLVisionAngularAndHorizDriveWeight;
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

	MLVisionAngularAndHorizDriveWeight mlVisionWeight;

	JoystickDriveWeight joystickDriveWeight;

	MovingWhileShooting movingWhileShooting;

	RotateToAngleWeight movingWhileShootingWeight;

	boolean autoTargetBool = false;

	private boolean startAuto = false;

	double autoSetAngle;

	public RobotContainer() {
		switch (Robot.getMode()) {
			case REAL:
				extensionSubsystem = new ExtensionSubsystem(new ExtensionIOSparkMax());
				gyro = new Gyro(new GyroIONavX());
				aprilTagVision1 = new AprilTagVision(new AprilTagVisionIOLimelight("limelight-front"), "-front");
				aprilTagVision2 = new AprilTagVision(new AprilTagVisionIOLimelight("limelight-back"), "-back");
			 	mlVision = new MLVision(new MLVisionIOLimelight());

				shooterSubsystem = new ShooterSubsystem(new ShooterIOSparkMax());
				// shooterSubsystem = new ShooterSubsystem(new ShooterIO(){});
				intakeSubsystem = new IntakeSubsystem(
						new IntakeIOSparkMax(() -> driveControllerHID.getPOV() == 0, ()->intakeSubsystem.isShooting(), ()->startAuto));
				climberSubsystem = new ClimberSubsystem(new ClimberIOSparkMax());
				// intakeSubsystem = new IntakeSubsystem(new IntakeIO(){});
				targetingSubsystem = new TargetingSubsystem(new TargetingIOSparkMax(), ()->angleOffset);
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
				mlVision = new MLVision(new MLVisionIOSim());

				intakeSubsystem = new IntakeSubsystem(new IntakeIOSim(() -> driveControllerHID.getPOV() == 0));
				climberSubsystem = new ClimberSubsystem(new ClimberIOSim());
				targetingSubsystem = new TargetingSubsystem(new TargetingIOSim(), ()->angleOffset);
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
				intakeSubsystem = new IntakeSubsystem(new IntakeIO() {
				});
				targetingSubsystem = new TargetingSubsystem(new TargetingIO() {
				},()->angleOffset);
				climberSubsystem = new ClimberSubsystem(new ClimberIO() {
				});
				mlVision = new MLVision(new MLVisionIO() {
				});
				break;
		}
		ArrayList<AprilTagVision> visions = new ArrayList<>();
		visions.add(aprilTagVision1);
		visions.add(aprilTagVision2);
		driveSubsystem = new DriveSubsystem(gyro, visions);

		
		// shooterSubsystem.setDefaultCommand(shooterSubsystem.setSpeedCommand(0, 0, 0, 0));
		joystickDriveWeight = new JoystickDriveWeight(driveController, gyro);
		DriveWeightCommand.addPersistentWeight(joystickDriveWeight);
		driveSubsystem.setDefaultCommand(new DriveWeightCommand().create(driveSubsystem));

		intakeSubsystem.setDefaultCommand(intakeSubsystem.intakeNoteCommand(0.8, 0.5, () -> intakeSubsystem.hasNote()));
		// intakeSubsystem.setDefaultCommand(intakeSubsystem.intakeCommand(0, 0));


		// targetingSubsystem.setDefaultCommand(targetingSubsystem.extendAndAngleSpeed(0, 0));
		
		// configure weights

		movingWhileShooting = new MovingWhileShooting(gyro, ()->getSpeakerPos(), driveSubsystem::getPose, 
		driveSubsystem::getChassisSpeeds, targetingSubsystem);
		// targetingSubsystem.setDefaultCommand(targetingSubsystem.anglePIDCommand(()->60));
		targetingSubsystem.setDefaultCommand(autoTargetMovingWhileShooting());

		// targetingSubsystem.setDefaultCommand(targetingSubsystem.anglePIDCommand(()->50));
		// targetingSubsystem.setDefaultCommand(
			// targetingSubsystem.anglePIDCommand(()->movingWhileShooting.getNewTargetingAngle()));

		shooterSubsystem.setDefaultCommand(shooterSubsystem.setSpeedPercentPID(()->0.5, ()->0.5, ()->0.4, ()->0.4, ()->get3dDistance(()->getSpeakerPos()) < 12));
		// shooterSubsystem.setDefaultCommand(
		// 	shooterSubsystem.setSpeedCommand(movingWhileShooting.speedToPercentOutput()));
		
		generateNamedCommands();

		rotateLockWeight = new RotateLockWeight(
				() -> (getAlliance() == Alliance.Blue
						? new Pose2d(FieldConstants.speakerPositionBlue, new Rotation2d())
						: new Pose2d(FieldConstants.speakerPositionRed, new Rotation2d())),
				driveSubsystem::getPose, gyro, () -> joystickDriveWeight.getTranslationalSpeed());//()->aprilTagVision1.getTx(), ()->aprilTagVision1.isTarget()

		movingWhileShootingWeight = new RotateToAngleWeight(()->(get3dDistance(() -> getSpeakerPos())< FieldConstants.fullCourtShootingRadius) 
			? (movingWhileShooting.getNewRobotAngle()) 
			: (getAngleToStash()),
		driveSubsystem::getPose, ()->joystickDriveWeight.getTranslationalSpeed(),
			"MovingWhileShooting", ()->false, driveSubsystem);

		autoDriveWeight = new AutoDriveWeight(
				() -> (getAlliance() == Alliance.Blue
						? new Pose2d(FieldConstants.ampPositionBlue, new Rotation2d(Math.PI / 2))
						: new Pose2d(FieldConstants.ampPositionRed, new Rotation2d(Math.PI / 2))),
				driveSubsystem::getPose, gyro);

		mlVisionWeight = new MLVisionAngularAndHorizDriveWeight(mlVision, gyro::getAngleRotation2d, ()->intakeSubsystem.hasNote());

		DashboardInit.init(driveSubsystem, driveController, visions, targetingSubsystem, shooterSubsystem, mlVision);
		configureBindings();


	}

	private void configureBindings() {

		// NOTE: if uncommenting commented out lines here,
		// DON'T use trigger methods in CommandXboxController.
		// Use triggers & HIDs to avoid >] COMMAND OVERRUN [<.

		// driveController.leftBumper().whileTrue(intakeSubsystem.intakeCommand(0, 0.8));//.alongWith(autoTarget())
		new Trigger(() -> operatorControllerHID.getXButton())
				.whileTrue(ampCommand()); // amp shot
		new Trigger(() -> driveControllerHID.getStartButton())
				.onTrue(driveSubsystem.resetGyroCommand());
		new Trigger(() -> driveControllerHID.getBackButton())
				.onTrue(driveSubsystem.resetOdometryAprilTag());
		new Trigger(() -> driveControllerHID.getLeftBumper())
				.whileTrue(generateIntakeCommand());
		new Trigger(() -> driveControllerHID.getYButton())
				.onTrue(new InstantCommand(() -> DriveWeightCommand.addWeight(autoDriveWeight)));
		new Trigger(() -> driveControllerHID.getYButton())
				.onFalse(new InstantCommand(() -> DriveWeightCommand.removeWeight(autoDriveWeight)));

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
				.andThen(new InstantCommand(() -> joystickDriveWeight.setWeight(0.5)))); // .alongWith(autoTarget()));
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
						() -> -operatorControllerHID.getLeftY(), () -> -operatorControllerHID.getRightY()));
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
		new Trigger(() -> driveControllerHID.getRightTriggerAxis() > 0.1)
				.onTrue(new InstantCommand(() -> DriveWeightCommand.addWeight(mlVisionWeight)));
		new Trigger(() -> driveControllerHID.getRightTriggerAxis() > 0.1)
		 		.onFalse(Commands.parallel(
					new InstantCommand(() -> DriveWeightCommand.removeWeight(mlVisionWeight)),
					new InstantCommand(() -> mlVisionWeight.resetMode())));
		new Trigger(() -> operatorControllerHID.getRightBumper())
				.whileTrue(
					extensionSubsystem.setExtensionPercentOutputCommand(TargetingConstants.extensionManualSpeed));
		new Trigger(() -> operatorControllerHID.getLeftBumper())
				.whileTrue(
					extensionSubsystem.setExtensionPercentOutputCommand(-TargetingConstants.extensionManualSpeed));

		// operatorController.rightBumper().whileTrue(targetingSubsystem.anglePIDCommand(30));
		// operatorController.leftBumper().whileTrue(targetingSubsystem.anglePIDCommand(30));

		new Trigger(() -> driveControllerHID.getBButton())
				.whileTrue(manualShotNoAngle(55, () -> !driveControllerHID.getBButton()));
		// driveController.y().whileTrue(intakeSubsystem.intakeCommand(-0.5, 0));

		// driveController.y().onTrue(driveSubsystem.resetOdometryAprilTag());

		new Trigger (() -> operatorControllerHID.getAButton())
				.whileTrue(manualShotNoAngle(41.8, () -> !operatorControllerHID.getAButton()));

		// driveController.y().whileTrue(() -> );
	}

	public void pidTriggers(){
		targetingSubsystem.setDefaultCommand(
			targetingSubsystem.anglePIDCommand(()->PIDUpdate.getSetpoint(),()->PIDUpdate.getPID() == PIDConstants.map.get("angle")));
		// shooterSubsystem.setDefaultCommand(
		// 	shooterSubsystem.setSpeedPercentPID(()->0, ()->PIDUpdate.getSetpoint(), ()->0, ()->0, 
		// 	()->PIDUpdate.getPID() == PIDConstants.map.get("bottomLeftShooter"))
		// );
		
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
				.andThen(driveSubsystem.driveDoubleConeCommand(() -> new ChassisSpeeds(), () -> new Translation2d()))
				.andThen(driveSubsystem.resetGyroCommand())
				.alongWith(new InstantCommand(()->Logger.recordOutput("AutoRun", DashboardInit.getAutoChooserCommand().getName())))
				.alongWith(new InstantCommand(()->{startAuto = true;}));
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

	private Command generateIntakeCommand() {
		return intakeSubsystem.intakeCommand(0, 0.8,
				() -> (shooterSubsystem.isSpeedAccurate(0.05) && targetingSubsystem.isAnglePositionAccurate(TargetingConstants.angleError)
						&& Math.toDegrees(Math.abs(SwerveAlgorithms.angleDistance(
								DriveWeightCommand.getAngle(), driveSubsystem.getPose().getRotation().getRadians()))) < 2.5));
	}

    private Command generateIntakeCommandAuto() {
			return new SequentialCommandGroup(new WaitUntilCommand(() -> !intakeSubsystem.hasNote()), new WaitCommand(0.1))
				.deadlineWith(intakeSubsystem.intakeCommand(0, 0.8,
				() -> (shooterSubsystem.isSpeedAccurate(0.3) 
					&& targetingSubsystem.isAnglePositionAccurate(2))).repeatedly());
	}

	private Command generateIntakeCommandAutoLowAccuracy() {
			return new SequentialCommandGroup(new WaitUntilCommand(() -> !intakeSubsystem.hasNote()), new WaitCommand(0.2)
				.deadlineWith(intakeSubsystem.intakeCommand(0, 0.8,
				() -> (shooterSubsystem.isSpeedAccurate(0.05) 
					&& targetingSubsystem.isAnglePositionAccurate(10)))).repeatedly());
	}

	public Command intakeNote(){
		return intakeSubsystem.intakeNoteCommand(0.8, 0.5, () -> intakeSubsystem.hasNote())
			.repeatedly().until(() -> intakeSubsystem.hasNote());
	}
	public Command generateIntakeNoRobot(double time, double indexSpeed){
		return intakeSubsystem.intakeCommand(0, indexSpeed,
				() -> (shooterSubsystem.isSpeedAccurate(0.1) 
				&& targetingSubsystem.isAnglePositionAccurate(6)), time);
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
		return targetingSubsystem.anglePIDCommand(targetAngle).alongWith(generateIntakeNoRobot(0, 0.8));
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

	public Command manualShot(double targetAngle, double robotAngleBlueRadians, double robotAngleRedRadians, BooleanSupplier cancelCondition) {
		return targetingSubsystem.anglePIDCommand(targetAngle).alongWith(new InstantCommand(() -> {
			RotateToAngleWeight weight = new RotateToAngleWeight(
					() -> (getAlliance()==Alliance.Blue?robotAngleBlueRadians:robotAngleRedRadians), 
					driveSubsystem::getPose, () -> joystickDriveWeight.getTranslationalSpeed(), "manualShot",
					cancelCondition, driveSubsystem);
			DriveWeightCommand.addWeight(weight);
		}).alongWith(generateIntakeCommand()));
	}

	public double get3dDistance(Supplier<Pose2d> speakerPos) {
		Logger.recordOutput("Drive/DistanceToSpeaker",
				new Translation3d(speakerPos.get().minus(driveSubsystem.getPose()).getX(),
						speakerPos.get().minus(driveSubsystem.getPose()).getY(), 2.11).getNorm());

		
		
		return new Translation3d(speakerPos.get().minus(driveSubsystem.getPose()).getX(),
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
		if(intakeSubsystem.hasNote() && get3dDistance(() -> getSpeakerPos()) < FieldConstants.fullCourtShootingRadius){
			return movingWhileShooting.getAngleFromDistance();
		} else if (intakeSubsystem.hasNote() && get3dDistance(() -> getSpeakerPos()) > FieldConstants.fullCourtShootingRadius && operatorController.getHID().getYButton()){
			return 40;
		} else {
			return 30;
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
			.alongWith(targetingSubsystem.anglePIDCommand(50));
	}

	public Command ampCommandNoShoot(){
		return shooterSubsystem.setSpeedPercentPID(()->0.03, ()->0.27,
			()->0.03, ()->0.27, ()->true)
			.alongWith(targetingSubsystem.anglePIDCommand(50));
	}

	public void generateNamedCommands(){
		// NamedCommands.registerCommand("", )
		double offset = 5.5;//5.5

		RotateToAngleWeight rotateToAngleWeightAuto = 
			new RotateToAngleWeight(()->autoSetAngle, ()->new Pose2d(0,0,new Rotation2d(gyro.getAngleRotation2d().getRadians())), ()->0.0,
			"AutoRotate", ()->false, driveSubsystem);

		RotateToAngleWeight movingWhileShootingWeightAuto = new RotateToAngleWeight(()->movingWhileShooting.getNewRobotAngle(),
			driveSubsystem::getPose, ()->0.0,
			"MovingWhileShootingAuto", ()->false, driveSubsystem);

		NamedCommands.registerCommand("Run Indexer", generateIntakeCommandAuto());
		NamedCommands.registerCommand("Run Intake", intakeNote());
		NamedCommands.registerCommand("AmpNoteShot", manualShotAuto(30));
		NamedCommands.registerCommand("AmpNoteShotQuad", manualShotAuto(34));
		NamedCommands.registerCommand("CenterFreeThrow", manualShotAuto(40));
		NamedCommands.registerCommand("SpeakerShot", manualShotAuto(55));
		NamedCommands.registerCommand("MidShot", manualShotAuto(30 + offset));
		NamedCommands.registerCommand("MidShotFromAmp", manualShotAuto(32 + offset));
		NamedCommands.registerCommand("StageShot", manualShotAuto(35+ offset));
		NamedCommands.registerCommand("CenterShot", manualShotAuto(35));
		NamedCommands.registerCommand("MidFarShot", manualShotAuto(35 + offset));

		NamedCommands.registerCommand("UnderStage", manualShotAuto(29));

		NamedCommands.registerCommand("CenterSourceSide", manualShotAuto(33));

		// NamedCommands.registerCommand("RunIndexerLowAccuracy", generateIntakeCommand());

		NamedCommands.registerCommand("AutoTargetAngle", targetingSubsystem.anglePIDCommand(() -> 
			movingWhileShooting.getAngleFromDistance(), 60, ()->autoTargetBool && extensionSubsystem.getExtensionPosition() < 20));

		NamedCommands.registerCommand("CloseShot", manualShotAuto(45));

		NamedCommands.registerCommand("4NoteAmp", manualShotAuto(35));
		
		NamedCommands.registerCommand("MLVisionAutonCommand", (new MLVisionAutoCommand2(() -> intakeSubsystem.hasNote(), mlVision, driveSubsystem, ()->gyro.getAngleRotation2d())).getCommand());
		NamedCommands.registerCommand("MLVisionAutonConstraintsCommand", mlVision.waitUntilMLCommand(4, 0));
		// NamedCommands.registerCommand("SetAngleFreeThrow", new InstantCommand(()->setAutoTargetAngle(Math.toRadians(-165))));

		NamedCommands.registerCommand("AmpSideRotate", driveSubsystem.rotateToAngleCommand(()->Math.toRadians((157 + (getAlliance() == Alliance.Red?180:0)))));
		NamedCommands.registerCommand("SetAngleFreeThrow", driveSubsystem.rotateToAngleCommand(()->(Math.toRadians(-180 + (getAlliance() == Alliance.Red?180:0)))));

		NamedCommands.registerCommand("RotateToGoal", movingWhileShootingWeightAuto.getAsCommand());

		NamedCommands.registerCommand("Triplet1stShot", manualShotAuto(30.5));

		// NamedCommands.registerCommand("90", new InstantCommand(()->setAutoTargetAngle(Math.toRadians(90))));

		// NamedCommands.registerCommand("TripletStage", new InstantCommand(()->setAutoTargetAngle(Math.toRadians(152))));
		
		//NamedCommands.registerCommand("MLVisionAutonStrafeCommand", (new MLVisionAutonStrafeCommand(mlVision, gyro::getAngleRotation2d, driveSubsystem, ()->intakeSubsystem.hasNote())
		//	.getCommand()));

	}
	private void setAutoTargetAngle(double angle){
		autoSetAngle = angle + (getAlliance() == Alliance.Red?90:0);
	}
	private void setAutoTargetAngleNoAlliance(double angle){
		autoSetAngle = angle + (getAlliance() == Alliance.Red?90:0);
	}
}