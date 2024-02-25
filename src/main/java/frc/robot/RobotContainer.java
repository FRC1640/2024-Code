// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.drive.DriveSubsystem;
import frc.lib.swerve.SwerveAlgorithms;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveDriveDimensions;
import frc.robot.Constants.TargetingConstants;
import frc.robot.sensors.Gyro.Gyro;
import frc.robot.sensors.Gyro.GyroIO;
import frc.robot.sensors.Gyro.GyroIONavX;
import frc.robot.sensors.Gyro.GyroIOSim;
import frc.robot.sensors.Vision.AprilTagVision.AprilTagVision;
import frc.robot.sensors.Vision.AprilTagVision.AprilTagVisionIO;
import frc.robot.sensors.Vision.AprilTagVision.AprilTagVisionIOSim;
import frc.robot.sensors.Vision.MLVision.MLVision;
import frc.robot.sensors.Vision.MLVision.MLVisionIO;
import frc.robot.sensors.Vision.MLVision.MLVisionIOSim;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveWeightCommand;
import frc.robot.subsystems.drive.DriveWeights.AutoDriveWeight;
import frc.robot.subsystems.drive.DriveWeights.JoystickDriveWeight;
import frc.robot.subsystems.drive.DriveWeights.MLVisionAngularAndHorizDriveWeight;
import frc.robot.subsystems.drive.DriveWeights.RotateLockWeight;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.targeting.TargetingIO;
import frc.robot.subsystems.targeting.TargetingIOSim;
import frc.robot.subsystems.targeting.TargetingIOSparkMax;
import frc.robot.subsystems.targeting.TargetingSubsystem;

public class RobotContainer {

    private Gyro gyro;
    private AprilTagVision aprilTagVision;
    private MLVision mlVision;

    private static DriveSubsystem driveSubsystem;
    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private ClimberSubsystem climberSubsystem;
    private TargetingSubsystem targetingSubsystem;

    RotateLockWeight rotateLockWeight;

    AutoDriveWeight autoDriveWeight;

    MLVisionAngularAndHorizDriveWeight mlVisionWeight;

    JoystickDriveWeight joystickDriveWeight;

    public RobotContainer() {
        switch (Robot.getMode()) {
            case REAL:
                gyro = new Gyro(new GyroIONavX());
                // aprilTagVision = new AprilTagVision(new AprilTagVisionIOLimelight("limelight"));
                // mlVision = new MLVision(new MLVisionIOLimelight());
                aprilTagVision = new AprilTagVision(new AprilTagVisionIOSim());
                mlVision = new MLVision(new MLVisionIOSim());

                shooterSubsystem = new ShooterSubsystem(new ShooterIOSparkMax());
                // shooterSubsystem = new ShooterSubsystem(new ShooterIO(){});
                intakeSubsystem = new IntakeSubsystem(new IntakeIOSparkMax(() -> driveController.povUp().getAsBoolean()));
                climberSubsystem = new ClimberSubsystem(new ClimberIO() {});
                // intakeSubsystem = new IntakeSubsystem(new IntakeIO(){});
                targetingSubsystem = new TargetingSubsystem(new TargetingIOSparkMax());
                // targetingSubsystem = new TargetingSubsystem(new TargetingIO() {});
                break;
            case SIM:
                gyro = new Gyro(new GyroIOSim(() -> Math.toDegrees(SwerveDriveDimensions.kinematics
                        .toChassisSpeeds(
                                driveSubsystem.getActualSwerveStates()).omegaRadiansPerSecond)));
                shooterSubsystem = new ShooterSubsystem(new ShooterIOSim());
                aprilTagVision = new AprilTagVision(new AprilTagVisionIOSim());
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
                aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {
                });
                intakeSubsystem = new IntakeSubsystem(new IntakeIO() {
                });
                targetingSubsystem = new TargetingSubsystem(new TargetingIO() {
                });
                climberSubsystem = new ClimberSubsystem(new ClimberIO() {
                });
                mlVision = new MLVision(new MLVisionIO(){});
                break;
        }
        driveSubsystem = new DriveSubsystem(gyro, aprilTagVision);
        DashboardInit.init(driveSubsystem, driveController, aprilTagVision);
        // shooterSubsystem.setDefaultCommand(shooterSubsystem.setSpeedCommand(0.8, 0.8, 0.7, 0.7));
        shooterSubsystem.setDefaultCommand(shooterSubsystem.setSpeedCommand(0, 0, 0, 0));
        joystickDriveWeight = new JoystickDriveWeight(driveController, gyro);
        DriveWeightCommand.addPersistentWeight(joystickDriveWeight);
        driveSubsystem.setDefaultCommand(new DriveWeightCommand().create(driveSubsystem));

        intakeSubsystem.setDefaultCommand(intakeSubsystem.intakeNoteCommand(0.5, 0.5, ()->intakeSubsystem.hasNote()));

        // targetingSubsystem.setDefaultCommand(targetingSubsystem
        //         .anglePIDCommand(
        //                 () -> -0.956635
        //                         * Math.toDegrees(
        //                                 Math.asin(-0.778591 * Units.metersToFeet(2.11)
        //                                         / Units.metersToFeet(get3dDistance(() -> getSpeakerPos())) - 0.22140))
        //                         -2.01438));

        targetingSubsystem.setDefaultCommand(targetingSubsystem.extendAndAngleSpeed(0, 0));
        //configure weights

        rotateLockWeight = new RotateLockWeight(
                () -> (getAlliance() == Alliance.Blue
                        ? new Pose2d(FieldConstants.speakerPositionBlue, new Rotation2d())
                        : new Pose2d(FieldConstants.speakerPositionRed, new Rotation2d())),
                driveSubsystem::getPose, gyro, ()->joystickDriveWeight.getTranslationalSpeed());

        autoDriveWeight = new AutoDriveWeight(
                () -> (getAlliance() == Alliance.Blue
                        ? new Pose2d(FieldConstants.ampPositionBlue, new Rotation2d(Math.PI / 2))
                        : new Pose2d(FieldConstants.ampPositionRed, new Rotation2d(Math.PI / 2))),
                driveSubsystem::getPose, gyro);
        
        mlVisionWeight = new MLVisionAngularAndHorizDriveWeight(mlVision, gyro::getAngleRotation2d);

        configureBindings();
    }

    private void configureBindings() {

        // driveController.x().whileTrue(shooterSubsystem.setSpeedCommand(0.1, 0.25, 0.1, 0.25)
        //         .alongWith(generateIntakeCommand())
        //                 .alongWith(targetingSubsystem.anglePIDCommand(60)));
                        // amp shot
        driveController.start().onTrue(driveSubsystem.resetGyroCommand());
        driveController.y().onTrue(driveSubsystem.resetOdometryAprilTag());
        driveController.leftBumper().whileTrue(generateIntakeCommand());
        driveController.rightBumper().onTrue(new InstantCommand(() -> DriveWeightCommand.addWeight(autoDriveWeight)))
                .whileTrue(shooterSubsystem.setSpeedCommand(0.1, 0.25, 0.1, 0.25)
                        .alongWith(targetingSubsystem.anglePIDCommand(60)));

        driveController.rightBumper()
                .onFalse(new InstantCommand(() -> DriveWeightCommand.removeWeight(autoDriveWeight))
                        .alongWith(Commands.race(
                                shooterSubsystem.setSpeedCommand(0.1, 0.25, 0.1, 0.25)
                                        .alongWith(targetingSubsystem.anglePIDCommand(60)),
                                new WaitCommand(2))));

        driveController.a().onTrue(new InstantCommand(() -> DriveWeightCommand.addWeight(rotateLockWeight))
                .andThen(new InstantCommand(()->joystickDriveWeight.setWeight(0.5))));
        driveController.a().onFalse(new InstantCommand(() -> DriveWeightCommand.removeWeight(rotateLockWeight))
                .andThen(new InstantCommand(()->joystickDriveWeight.setWeight(1))));
        operatorController.leftTrigger()
                .whileTrue(targetingSubsystem.setAnglePercentOutputCommand(-TargetingConstants.angleManualSpeed));
        operatorController.rightTrigger()
                .whileTrue(targetingSubsystem.setAnglePercentOutputCommand(TargetingConstants.angleManualSpeed));
        new Trigger(() -> Math.abs(operatorController.getLeftY()) > 0.1 ||
                Math.abs(operatorController.getRightY()) > 0.1)
                .whileTrue(climberSubsystem.setSpeedCommand(
                        ()->-operatorController.getLeftY() * 0.5, ()->-operatorController.getRightY() * 0.5));
        new Trigger(() -> intakeSubsystem.hasNote())
                .onTrue(new InstantCommand(
                        () -> driveController.getHID().setRumble(RumbleType.kBothRumble, 0.3)));
        new Trigger(() -> intakeSubsystem.hasNote())
                .onFalse(new InstantCommand(
                        () -> driveController.getHID().setRumble(RumbleType.kBothRumble, 0.0)));
		//operatorController.a().onTrue(climberSubsystem.climbRoutineCommand()); gonna change to be a separate command that runs this after lining up
       
        driveController.rightTrigger().onTrue(new InstantCommand(()->
             DriveWeightCommand.addWeight(mlVisionWeight)));
        driveController.rightTrigger().onFalse(new InstantCommand(()->
             DriveWeightCommand.removeWeight(mlVisionWeight)));
        operatorController.rightBumper().whileTrue(targetingSubsystem.setExtensionPercentOutputCommand(TargetingConstants.extensionManualSpeed));
        operatorController.leftBumper().whileTrue(targetingSubsystem.setExtensionPercentOutputCommand(-TargetingConstants.extensionManualSpeed));
        // operatorController.x().onTrue(targetingSubsystem.extend(0.5));
        operatorController.pov(90).whileTrue(climbRoutineCommand(() -> driveSubsystem.getPose()));
    }

    public Command getAutonomousCommand() {
        return DashboardInit.getAutoChooserCommand()
                .andThen(driveSubsystem.driveDoubleConeCommand(() -> new ChassisSpeeds(), () -> new Translation2d()));
    }

    public void removeAllDefaultCommands() {
        driveSubsystem.removeDefaultCommand();
        shooterSubsystem.removeDefaultCommand();
        intakeSubsystem.removeDefaultCommand();
        targetingSubsystem.removeDefaultCommand();
        climberSubsystem.removeDefaultCommand();
    }
//i put a bunch of 'static's everywhere including the drive subsystem so everything might explode but i thought id make note
    private static Alliance getAlliance() {
        if (DriverStation.getAlliance().isPresent()) {
            return DriverStation.getAlliance().get();
        }
        return Alliance.Blue;
    }

    public static Pose2d getNearestStage(){ //will probably end up changing first two keywords we'll see
		Pose2d[] list = getAlliance() == Alliance.Blue?FieldConstants.blueStages:FieldConstants.redStages;
		double best = 999999;
		Pose2d bestPose = new Pose2d();
		for (Pose2d position : list){
			if (position.getTranslation().getDistance(driveSubsystem.getPose().getTranslation()) < best){
				best = position.getTranslation().getDistance(driveSubsystem.getPose().getTranslation());
				bestPose = position;
			}
			
		}
		return bestPose;
    }

    private Command generateIntakeCommand() {
        return intakeSubsystem.intakeCommand(0, 0.5,
                () -> (shooterSubsystem.isSpeedAccurate(0.05) && targetingSubsystem.isAnglePositionAccurate(7)
                && Math.toDegrees(Math.abs(SwerveAlgorithms.angleDistance(
                        DriveWeightCommand.getAngle(),gyro.getAngleRotation2d().getRadians()))) < 3));
    }

    public Pose2d getSpeakerPos() {
        return (getAlliance() == Alliance.Blue
                ? new Pose2d(FieldConstants.speakerPositionBlue, new Rotation2d())
                : new Pose2d(FieldConstants.speakerPositionRed, new Rotation2d()));
    }

    public double get3dDistance(Supplier<Pose2d> speakerPos) {
        Logger.recordOutput("Drive/DistanceToSpeaker",
                new Translation3d(speakerPos.get().minus(driveSubsystem.getPose()).getX(),
                        speakerPos.get().minus(driveSubsystem.getPose()).getY(), 2.11).getNorm());
        return new Translation3d(speakerPos.get().minus(driveSubsystem.getPose()).getX(),
                speakerPos.get().minus(driveSubsystem.getPose()).getY(), 2.11).getNorm();

    }

    public Command climbRoutineCommand(Supplier<Pose2d> currentLocation){
        Command c = new Command(){
                AutoDriveWeight AutoDrive = new AutoDriveWeight(() -> RobotContainer.getNearestStage(), currentLocation, gyro);
                @Override
                public void initialize(){
                        targetingSubsystem.anglePIDCommand(() -> 110);
                }
    
                @Override
                public void execute(){
                        DriveWeightCommand.addWeight(AutoDrive); //some work needs to be done to stop this when we want it to stop
                        targetingSubsystem.extensionPIDCommand(0);  //change this to the value we want
                        climberSubsystem.climberPIDCommand(90,90);

                }

                @Override
                public void end(boolean interrupted){
                        targetingSubsystem.anglePIDCommand(() -> 60);
                        targetingSubsystem.extensionPIDCommand(0);
                        climberSubsystem.setSpeedCommand(0, 0);
                        DriveWeightCommand.removeWeight(AutoDrive);
                }
            };
            return c;
    }
}