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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.drive.DriveSubsystem;
import frc.lib.swerve.SwerveAlgorithms;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveDriveDimensions;
import frc.robot.Constants.TargetingConstants;
import frc.robot.sensors.Gyro.Gyro;
import frc.robot.sensors.Gyro.GyroIO;
import frc.robot.sensors.Gyro.GyroIONavX;
import frc.robot.sensors.Gyro.GyroIOSim;
import frc.robot.sensors.Vision.AprilTagVision.AprilTagVision;
import frc.robot.sensors.Vision.AprilTagVision.AprilTagVisionIO;
import frc.robot.sensors.Vision.AprilTagVision.AprilTagVisionIOSim;
import frc.robot.sensors.Vision.AprilTagVision.AprilTagVisionIOLimelight;
import frc.robot.sensors.Vision.MLVision.MLVision;
import frc.robot.sensors.Vision.MLVision.MLVisionIO;
import frc.robot.sensors.Vision.MLVision.MLVisionIOLimelight;
import frc.robot.sensors.Vision.MLVision.MLVisionIOSim;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOSparkMax;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveWeightCommand;
import frc.robot.subsystems.drive.DriveWeights.AutoDriveWeight;
import frc.robot.subsystems.drive.DriveWeights.JoystickDriveWeight;
import frc.robot.subsystems.drive.DriveWeights.MLVisionAngularAndHorizDriveWeight;
import frc.robot.subsystems.drive.DriveWeights.RotateLockWeight;
import frc.robot.subsystems.drive.DriveWeights.RotateLockWeightTrack;
import frc.robot.subsystems.drive.DriveWeights.RotateToAngleWeight;
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
	private ShooterSubsystem shooterSubsystem;
	private IntakeSubsystem intakeSubsystem;
	private ClimberSubsystem climberSubsystem;
	private TargetingSubsystem targetingSubsystem;

	RotateLockWeight rotateLockWeight;

	// RotateToAngleWeight rotateLockWeight;

	AutoDriveWeight autoDriveWeight;

	MLVisionAngularAndHorizDriveWeight mlVisionWeight;

	JoystickDriveWeight joystickDriveWeight;

	MovingWhileShooting movingWhileShooting;

	RotateToAngleWeight movingWhileShootingWeight;

	boolean autoTargetBool = false;

	private boolean startAuto = false;

	public RobotContainer() {
		switch (Robot.getMode()) {
			case REAL:
				gyro = new Gyro(new GyroIONavX());
				aprilTagVision1 = new AprilTagVision(new AprilTagVisionIOLimelight("limelight-front"), "-front");
				aprilTagVision2 = new AprilTagVision(new AprilTagVisionIOLimelight("limelight-back"), "-back");
			 	mlVision = new MLVision(new MLVisionIOLimelight());

				shooterSubsystem = new ShooterSubsystem(new ShooterIOSparkMax());
				// shooterSubsystem = new ShooterSubsystem(new ShooterIO(){});
				intakeSubsystem = new IntakeSubsystem(
						new IntakeIOSparkMax(() -> driveController.povUp().getAsBoolean(), ()->intakeSubsystem.isShooting() || driveController.y().getAsBoolean(), ()->startAuto));
				climberSubsystem = new ClimberSubsystem(new ClimberIOSparkMax());
				// intakeSubsystem = new IntakeSubsystem(new IntakeIO(){});
				targetingSubsystem = new TargetingSubsystem(new TargetingIOSparkMax());
				// targetingSubsystem = new TargetingSubsystem(new TargetingIO() {});
				break;
			case SIM:
				gyro = new Gyro(new GyroIOSim(() -> Math.toDegrees(SwerveDriveDimensions.kinematics
						.toChassisSpeeds(
							driveSubsystem.getActualSwerveStates()).omegaRadiansPerSecond)));
				shooterSubsystem = new ShooterSubsystem(new ShooterIOSim());
				aprilTagVision1 = new AprilTagVision(new AprilTagVisionIOSim("limelight-front"),"-front");
				aprilTagVision2 = new AprilTagVision(new AprilTagVisionIOSim("limelight-back"),"-back");
				mlVision = new MLVision(new MLVisionIOSim());

				intakeSubsystem = new IntakeSubsystem(new IntakeIOSim(() -> driveController.povUp().getAsBoolean()));
				climberSubsystem = new ClimberSubsystem(new ClimberIOSim());
				targetingSubsystem = new TargetingSubsystem(new TargetingIOSim());
				break;

			default:
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
				});
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
		targetingSubsystem.setDefaultCommand(autoTarget());

		// targetingSubsystem.setDefaultCommand(targetingSubsystem.anglePIDCommand(()->50));
		// targetingSubsystem.setDefaultCommand(
			// targetingSubsystem.anglePIDCommand(()->movingWhileShooting.getNewTargetingAngle()));

		shooterSubsystem.setDefaultCommand(shooterSubsystem.setSpeedPercentPID(()->0.5, ()->0.5, ()->0.4, ()->0.4, ()->get3dDistance(()->getSpeakerPos()) < 10.249));
		// shooterSubsystem.setDefaultCommand(
		// 	shooterSubsystem.setSpeedCommand(movingWhileShooting.speedToPercentOutput()));
		
		generateNamedCommands();

		rotateLockWeight = new RotateLockWeight(
				() -> (getAlliance() == Alliance.Blue
						? new Pose2d(FieldConstants.speakerPositionBlue, new Rotation2d())
						: new Pose2d(FieldConstants.speakerPositionRed, new Rotation2d())),
				driveSubsystem::getPose, gyro, () -> joystickDriveWeight.getTranslationalSpeed());//()->aprilTagVision1.getTx(), ()->aprilTagVision1.isTarget()

		movingWhileShootingWeight = new RotateToAngleWeight(()->movingWhileShooting.getNewRobotAngle(),
		driveSubsystem::getPose, ()->joystickDriveWeight.getTranslationalSpeed(),
			"MovingWhileShooting", ()->false);

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

		driveController.x().whileTrue(ampCommand());
		// amp shot
		driveController.start().onTrue(driveSubsystem.resetGyroCommand());
		operatorController.y().onTrue(driveSubsystem.resetOdometryAprilTag());
		driveController.leftBumper().whileTrue(generateIntakeCommand());//.alongWith(autoTarget())
		// driveController.rightBumper().onTrue(new InstantCommand(() -> DriveWeightCommand.addWeight(autoDriveWeight)))
		// 		.whileTrue(ampCommandNoShoot());

		// driveController.rightBumper()
		// 		.onFalse(new InstantCommand(() -> DriveWeightCommand.removeWeight(autoDriveWeight))
		// 				.alongWith(Commands.race(
		// 						ampCommandNoShoot(),
		// 						new WaitCommand(ShooterConstants.waitTime))));

		driveController.a().onTrue(new InstantCommand(() -> DriveWeightCommand.addWeight(rotateLockWeight))
				.andThen(new InstantCommand(() -> joystickDriveWeight.setWeight(0.5))));
				// .alongWith(autoTarget()));
		driveController.a().onFalse(new InstantCommand(() -> DriveWeightCommand.removeWeight(rotateLockWeight))
				.andThen(new InstantCommand(() -> joystickDriveWeight.setWeight(1))));
				// .alongWith(Commands.race(autoTarget(), new WaitCommand(ShooterConstants.waitTime))));

		
		operatorController.leftTrigger()
				.whileTrue(targetingSubsystem.setAnglePercentOutputCommand(-0.03));
		operatorController.rightTrigger()
				.whileTrue(targetingSubsystem.setAnglePercentOutputCommand(0.03));

		driveController.povDown().onTrue(new InstantCommand(()->toggleAutoTarget(true)));


		new Trigger(() -> Math.abs(operatorController.getLeftY()) > 0.1 ||
				Math.abs(operatorController.getRightY()) > 0.1)
				.whileTrue(climberSubsystem.setSpeedCommand(
						() -> -operatorController.getLeftY(), () -> -operatorController.getRightY()));
		new Trigger(() -> intakeSubsystem.hasNote())
				.onTrue(new InstantCommand(
						() -> driveController.getHID().setRumble(RumbleType.kBothRumble, 1)));
		new Trigger(() -> intakeSubsystem.hasNote())
				.onFalse(new InstantCommand(
						() -> driveController.getHID().setRumble(RumbleType.kBothRumble, 0.0)));

		driveController.rightTrigger().onTrue(new InstantCommand(() -> DriveWeightCommand.addWeight(mlVisionWeight)));
		driveController.rightTrigger()
		 				.onFalse(Commands.parallel(new InstantCommand(() -> DriveWeightCommand.removeWeight(mlVisionWeight)), new InstantCommand(() -> mlVisionWeight.resetMode())));
		// operatorController.rightBumper().whileTrue(
		// 		targetingSubsystem.setExtensionPercentOutputCommand(TargetingConstants.extensionManualSpeed));
		// operatorController.leftBumper().whileTrue(
		// 		targetingSubsystem.setExtensionPercentOutputCommand(-TargetingConstants.extensionManualSpeed));

		operatorController.rightBumper().whileTrue(targetingSubsystem.anglePIDCommand(30));
		operatorController.leftBumper().whileTrue(targetingSubsystem.anglePIDCommand(30));

		driveController.b().whileTrue(manualShotNoAngle(60,
			()->!driveController.b().getAsBoolean()));
		// operatorController.x().onTrue(targetingSubsystem.extend(0.5));
		// driveController.y().whileTrue(intakeSubsystem.intakeCommand(-0.5, 0));

		driveController.y().onTrue(driveSubsystem.resetOdometryAprilTag());


		
	}

	public void pidTriggers(){
		// targetingSubsystem.setDefaultCommand(
		// 	targetingSubsystem.anglePIDCommand(()->PIDUpdate.getSetpoint(),()->PIDUpdate.getPID() == PIDConstants.map.get("angle")));
		shooterSubsystem.setDefaultCommand(
			shooterSubsystem.setSpeedPercentPID(()->0, ()->PIDUpdate.getSetpoint(), ()->0, ()->0, 
			()->PIDUpdate.getPID() == PIDConstants.map.get("bottomLeftShooter"))
		);
		
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
				() -> (shooterSubsystem.isSpeedAccurate(0.05) && targetingSubsystem.isAnglePositionAccurate(Constants.TargetingConstants.angleError)
						&& Math.toDegrees(Math.abs(SwerveAlgorithms.angleDistance(
								DriveWeightCommand.getAngle(), driveSubsystem.getPose().getRotation().getRadians()))) < 3));
	}

    private Command generateIntakeCommandAuto() {
			return new SequentialCommandGroup(new WaitUntilCommand(() -> !intakeSubsystem.hasNote()), new WaitCommand(0.75))
				.deadlineWith(intakeSubsystem.intakeCommand(0, 0.8,
				() -> (shooterSubsystem.isSpeedAccurate(0.05) 
					&& targetingSubsystem.isAnglePositionAccurate(Constants.TargetingConstants.angleError))).repeatedly());
	}

	public Command intakeNote(){
		return intakeSubsystem.intakeNoteCommand(0.8, 0.5, () -> intakeSubsystem.hasNote())
			.repeatedly().until(() -> intakeSubsystem.hasNote());
	}
	public Command generateIntakeNoRobot(double time){
		return intakeSubsystem.intakeCommand(0, 0.2,
				() -> (shooterSubsystem.isSpeedAccurate(0.1) 
				&& targetingSubsystem.isAnglePositionAccurate(Constants.TargetingConstants.angleError)), time);
	}

	public Pose2d getSpeakerPos() {
		return (getAlliance() == Alliance.Blue
				? new Pose2d(FieldConstants.speakerPositionBlue, new Rotation2d())
				: new Pose2d(FieldConstants.speakerPositionRed, new Rotation2d()));
	}

	public Command manualShotNoAngle(double targetAngle, BooleanSupplier cancelCondition) {
		return targetingSubsystem.anglePIDCommand(targetAngle).alongWith(generateIntakeNoRobot(400));
	}

	public Command manualShotAuto(double targetAngle) {
		return targetingSubsystem.anglePIDCommand(targetAngle).repeatedly();
	}

	public Command manualShot(double targetAngle, double robotAngleBlueRadians, double robotAngleRedRadians, BooleanSupplier cancelCondition) {
		return targetingSubsystem.anglePIDCommand(targetAngle).alongWith(new InstantCommand(() -> {
			RotateToAngleWeight weight = new RotateToAngleWeight(
					() -> (getAlliance()==Alliance.Blue?robotAngleBlueRadians:robotAngleRedRadians), 
					driveSubsystem::getPose, () -> joystickDriveWeight.getTranslationalSpeed(), "manualShot",
					cancelCondition);
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
	public Command autoTarget(){
		return targetingSubsystem.anglePIDCommand(() -> 
			targetingSubsystem.distToAngle(()->get3dDistance(() -> getSpeakerPos())), 60, ()->autoTargetBool);

	}

	public Command ampCommand(){
		return shooterSubsystem.setSpeedPercentPID(()->0.03, ()->0.27,
			()->0.03, ()->0.27, ()->true)
			.alongWith(generateIntakeNoRobot(500))
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
		NamedCommands.registerCommand("Run Indexer", generateIntakeCommandAuto());
		NamedCommands.registerCommand("Run Intake", intakeNote());
		NamedCommands.registerCommand("AmpNoteShot", manualShotAuto(32 + offset));
		NamedCommands.registerCommand("SpeakerShot", manualShotAuto(60));
		NamedCommands.registerCommand("MidShot", manualShotAuto(37 + offset));
		NamedCommands.registerCommand("MidShotFromAmp", manualShotAuto(35 + offset));
		NamedCommands.registerCommand("StageShot", manualShotAuto(33+ offset));
		NamedCommands.registerCommand("CenterShot", manualShotAuto(31 + offset));
		NamedCommands.registerCommand("MidFarShot", manualShotAuto(35 + offset));
	}
}