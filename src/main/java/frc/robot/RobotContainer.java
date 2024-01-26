// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.drive.DriveSubsystem;
import frc.robot.Constants.SwerveDriveDimensions;
import frc.robot.sensors.Gyro.Gyro;
import frc.robot.sensors.Gyro.GyroIO;
import frc.robot.sensors.Gyro.GyroIONavX;
import frc.robot.sensors.Gyro.GyroIOSim;
import frc.robot.sensors.Vision.AprilTagVision;
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
import frc.robot.subsystems.drive.DriveWeights.RotateLockWeight;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RobotContainer {

  private Gyro gyro;
  private AprilTagVision aprilTagVision;
  private DriveSubsystem driveSubsystem;
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private ShooterSubsystem shooterSubsystem;
  private IntakeSubsystem intakeSubsystem;
  public RobotContainer() {
      switch (Robot.getMode()) {
        case REAL:
          gyro = new Gyro(new GyroIONavX());
          aprilTagVision = new AprilTagVision(new AprilTagVisionIOLimelight());
          shooterSubsystem = new ShooterSubsystem(new ShooterIOSparkMax());
          intakeSubsystem = new IntakeSubsystem(new IntakeIOSparkMax());
          break;
        case SIM:
                gyro = new Gyro(new GyroIOSim(() -> Math.toDegrees(SwerveDriveDimensions.kinematics
                        .toChassisSpeeds(
                                driveSubsystem.getActualSwerveStates()).omegaRadiansPerSecond)));
                shooterSubsystem = new ShooterSubsystem(new ShooterIO(){});
                aprilTagVision = new AprilTagVision(new AprilTagVisionIOSim());
                intakeSubsystem = new IntakeSubsystem(new IntakeIOSim());
                break;

         default:
                gyro = new Gyro(new GyroIO(){});
                shooterSubsystem = new ShooterSubsystem(new ShooterIO(){});
                aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {});
                intakeSubsystem = new IntakeSubsystem(new IntakeIO() {});
                break;
        }
        driveSubsystem = new DriveSubsystem(gyro, aprilTagVision);
        DashboardInit.init(driveSubsystem, driveController, aprilTagVision);
        shooterSubsystem.setDefaultCommand(shooterSubsystem.setSpeedCommand(0.5, 0.5));
        DriveWeightCommand.addWeight(new JoystickDriveWeight(driveController, gyro));
        driveSubsystem.setDefaultCommand(new DriveWeightCommand().create(driveSubsystem));
        
        configureBindings();
    }

    private void configureBindings() {
        driveController.start().onTrue(driveSubsystem.resetGyroCommand());
        driveController.leftBumper().onTrue(driveSubsystem.resetOdometryCommand(new Pose2d(0, 0, new Rotation2d(0))));
        new Trigger(() -> !intakeSubsystem.hasNote()).whileTrue(intakeSubsystem.intakeCommand(1.0, 1.0));
        driveController.rightBumper().whileTrue(shooterSubsystem.setSpeedCommand(1, 1));
        driveController.rightBumper().onTrue(new InstantCommand(()->
            DriveWeightCommand.addWeight(new AutoDriveWeight(()->DriverStation.getAlliance().get() == Alliance.Blue ?
            new Pose2d(1.859, 7.803, new Rotation2d(Math.PI/2)):new Pose2d(14.667, 7.8, new Rotation2d(Math.PI/2)), driveSubsystem::getPose, gyro))));
        driveController.rightBumper().onFalse(new InstantCommand(()->
            DriveWeightCommand.removeWeight("AutoDriveWeight")));

        driveController.a().onTrue(new InstantCommand(()->
            DriveWeightCommand.addWeight(new RotateLockWeight(()->DriverStation.getAlliance().get() == Alliance.Blue ?
            new Pose2d(1.328, 5.555, new Rotation2d()):new Pose2d(15.214, 5.555, new Rotation2d()), driveSubsystem::getPose, gyro))));
        driveController.a().onFalse(new InstantCommand(()->
            DriveWeightCommand.removeWeight("RotateLockWeight")));
        //  driveController, gyro, new Pose2d(0,0,new Rotation2d(0))));
    }

    public Command getAutonomousCommand() {
        return DashboardInit.getAutoChooserCommand().andThen(driveSubsystem.driveDoubleConeCommand(()->new ChassisSpeeds(), ()->new Translation2d()));
    }

    public void removeAllDefaultCommands(){
        driveSubsystem.removeDefaultCommand();
        shooterSubsystem.removeDefaultCommand();
        intakeSubsystem.removeDefaultCommand();
    }
}