// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import java.time.Instant;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.drive.DriveSubsystem;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveDriveDimensions;
import frc.robot.Constants.TargetingConstants;
import frc.robot.sensors.Gyro.Gyro;
import frc.robot.sensors.Gyro.GyroIO;
import frc.robot.sensors.Gyro.GyroIONavX;
import frc.robot.sensors.Gyro.GyroIOSim;
import frc.robot.sensors.Vision.AprilTagVision;
import frc.robot.sensors.Vision.MLVision;
import frc.robot.sensors.Vision.MLVisionIOLimelight;
import frc.robot.sensors.Vision.MLVisionIOSim;
import frc.robot.sensors.Vision.AprilTagVisionIO;
import frc.robot.sensors.Vision.AprilTagVisionIOLimelight;
import frc.robot.sensors.Vision.AprilTagVisionIOSim;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.drive.DriveWeightCommand;
import frc.robot.subsystems.drive.DriveWeights.AutoDriveWeight;
import frc.robot.subsystems.drive.DriveWeights.JoystickDriveWeight;
import frc.robot.subsystems.drive.DriveWeights.MLVisionAngularAndHorizDriveWeight;
import frc.robot.subsystems.drive.DriveWeights.MLVisionRotationDriveWeight;
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

    private DriveSubsystem driveSubsystem;
    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private TargetingSubsystem targetingSubsystem;

    RotateLockWeight rotateLockWeight;

    AutoDriveWeight autoDriveWeight;

    MLVisionAngularAndHorizDriveWeight mlVisionWeight;

    JoystickDriveWeight joystickDriveWeight;

    public RobotContainer() {
        switch (Robot.getMode()) {
            case REAL:
                gyro = new Gyro(new GyroIONavX());
                aprilTagVision = new AprilTagVision(new AprilTagVisionIOLimelight());
                mlVision = new MLVision(new MLVisionIOLimelight());
                // shooterSubsystem = new ShooterSubsystem(new ShooterIOSparkMax());
                shooterSubsystem = new ShooterSubsystem(new ShooterIO(){});
                //intakeSubsystem = new IntakeSubsystem(new IntakeIOSparkMax());
                intakeSubsystem = new IntakeSubsystem(new IntakeIO(){});
                // targetingSubsystem = new TargetingSubsystem(new TargetingIOSparkMax());
                targetingSubsystem = new TargetingSubsystem(new TargetingIO() {});
                break;
            case SIM:
                gyro = new Gyro(new GyroIOSim(() -> Math.toDegrees(SwerveDriveDimensions.kinematics
                        .toChassisSpeeds(
                                driveSubsystem.getActualSwerveStates()).omegaRadiansPerSecond)));
                shooterSubsystem = new ShooterSubsystem(new ShooterIOSim());
                aprilTagVision = new AprilTagVision(new AprilTagVisionIOSim());
                mlVision = new MLVision(new MLVisionIOSim());

                intakeSubsystem = new IntakeSubsystem(new IntakeIOSim());
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
                mlVision = new MLVision(new MLVisionIOLimelight());

                break;
        }
        driveSubsystem = new DriveSubsystem(gyro, aprilTagVision);
        DashboardInit.init(driveSubsystem, driveController, aprilTagVision);
        shooterSubsystem.setDefaultCommand(shooterSubsystem.setSpeedCommand(0.8, 0.8, 0.7, 0.7));
        joystickDriveWeight = new JoystickDriveWeight(driveController, gyro);
        DriveWeightCommand.addPersistentWeight(joystickDriveWeight);
        driveSubsystem.setDefaultCommand(new DriveWeightCommand().create(driveSubsystem));

        intakeSubsystem.setDefaultCommand(intakeSubsystem.intakeCommand(1.0, 1.0));

        targetingSubsystem.setDefaultCommand(targetingSubsystem
                .targetFocusPosition(
                        () -> -0.956635
                                * Math.toDegrees(
                                        Math.asin(-0.778591 * Units.metersToFeet(2.11)
                                                / Units.metersToFeet(get3dDistance(() -> getSpeakerPos())) - 0.22140))
                                -2.01438));


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
        
        mlVisionWeight = new MLVisionAngularAndHorizDriveWeight(mlVision, driveController, gyro::getAngleRotation2d);

        configureBindings();
    }

    private void configureBindings() {

        driveController.x().whileTrue(shooterSubsystem.setSpeedCommand(0.1, 0.25, 0.1, 0.25)
                .alongWith(new InstantCommand(() -> generateIntakeCommand().schedule())
                        .alongWith(targetingSubsystem.targetFocusPosition(60)))); // amp shot
        driveController.start().onTrue(driveSubsystem.resetGyroCommand());
        driveController.y().onTrue(driveSubsystem.resetOdometryCommand(new
        Pose2d(0, 0, new Rotation2d(0))));
        driveController.leftBumper().whileTrue(new InstantCommand(() -> generateIntakeCommand().schedule()));
        new Trigger(() -> intakeSubsystem.hasNote()).whileTrue(intakeSubsystem.intakeCommand(0, 0));
        // driveController.rightBumper().whileTrue(shooterSubsystem.setSpeedCommand(1,
        // 1));
        driveController.rightBumper().onTrue(new InstantCommand(() -> DriveWeightCommand.addWeight(autoDriveWeight)))
                .whileTrue(shooterSubsystem.setSpeedCommand(0.1, 0.25, 0.1, 0.25)
                        .alongWith(targetingSubsystem.targetFocusPosition(60)));

        driveController.rightBumper()
                .onFalse(new InstantCommand(() -> DriveWeightCommand.removeWeight(autoDriveWeight))
                        .alongWith(Commands.race(
                                shooterSubsystem.setSpeedCommand(0.1, 0.25, 0.1, 0.25)
                                        .alongWith(targetingSubsystem.targetFocusPosition(60)),
                                new WaitCommand(2))));

        driveController.a().onTrue(new InstantCommand(() -> DriveWeightCommand.addWeight(rotateLockWeight))
                .andThen(new InstantCommand(()->joystickDriveWeight.setWeight(0.5))));
        driveController.a().onFalse(new InstantCommand(() -> DriveWeightCommand.removeWeight(rotateLockWeight))
                .andThen(new InstantCommand(()->joystickDriveWeight.setWeight(1))));
        // driveController, gyro, new Pose2d(0,0,new Rotation2d(0))));
        operatorController.leftTrigger()
                .whileTrue(targetingSubsystem.setSpeedCommand(-TargetingConstants.targetingManualSpeed));
        operatorController.rightTrigger()
                .whileTrue(targetingSubsystem.setSpeedCommand(TargetingConstants.targetingManualSpeed));
        new Trigger(() -> intakeSubsystem.hasNote())
                .onTrue(new InstantCommand(
                        () -> driveController.getHID().setRumble(RumbleType.kBothRumble, 0.3)));
        new Trigger(() -> intakeSubsystem.hasNote())
                .onFalse(new InstantCommand(
                        () -> driveController.getHID().setRumble(RumbleType.kBothRumble, 0.0)));
       
        driveController.rightTrigger().onTrue(new InstantCommand(()->
             DriveWeightCommand.addWeight(mlVisionWeight)));
        driveController.rightTrigger().onFalse(new InstantCommand(()->
             DriveWeightCommand.removeWeight(mlVisionWeight)));
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
    }

    private Alliance getAlliance() {
        if (DriverStation.getAlliance().isPresent()) {
            return DriverStation.getAlliance().get();
        }
        return Alliance.Blue;
    }

    private Command generateIntakeCommand() {
        return intakeSubsystem.intakeCommand(0, 0.5,
                () -> shooterSubsystem.isSpeedAccurate(0.05) && targetingSubsystem.isPositionAccurate(0.1));
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
}