// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.drive.DriveSubsystem;
import frc.robot.Constants.SwerveDriveDimensions;
import frc.robot.Robot.TestMode;
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
import frc.robot.subsystems.drive.DriveWeightCommand;
import frc.robot.subsystems.drive.DriveWeights.AutoDriveWeight;
import frc.robot.subsystems.drive.DriveWeights.JoystickDriveWeight;
import frc.robot.subsystems.drive.DriveWeights.MLVisionRotationDriveWeight;
import frc.robot.subsystems.drive.DriveWeights.RotateLockWeight;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RobotContainer {

  private Gyro gyro;
  private AprilTagVision aprilTagVision;
  private MLVision mlVision;

  private DriveSubsystem driveSubsystem;
  private final CommandXboxController driveController = new CommandXboxController(0);
  private ShooterSubsystem shooterSubsystem;
  public RobotContainer() {
      switch (Robot.getMode()) {
        case REAL:
          gyro = new Gyro(new GyroIONavX());
          aprilTagVision = new AprilTagVision(new AprilTagVisionIOLimelight());
          mlVision = new MLVision(new MLVisionIOLimelight());
          shooterSubsystem = new ShooterSubsystem(new ShooterIOSparkMax());
          break;
        case SIM:
                gyro = new Gyro(new GyroIOSim(() -> Math.toDegrees(SwerveDriveDimensions.kinematics
                        .toChassisSpeeds(
                                driveSubsystem.getActualSwerveStates()).omegaRadiansPerSecond)));
                shooterSubsystem = new ShooterSubsystem(new ShooterIO(){});
                aprilTagVision = new AprilTagVision(new AprilTagVisionIOSim());
                mlVision = new MLVision(new MLVisionIOSim());

                break;

         default:
                gyro = new Gyro(new GyroIO(){});
                shooterSubsystem = new ShooterSubsystem(new ShooterIO(){});
                aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {});
                mlVision = new MLVision(new MLVisionIOLimelight());

                break;
        }
        driveSubsystem = new DriveSubsystem(gyro, aprilTagVision);
        DashboardInit.init(driveSubsystem, driveController);
        shooterSubsystem.setDefaultCommand(shooterSubsystem.setSpeedCommand(0.5, 0.5));
        driveSubsystem.setDefaultCommand(new DriveWeightCommand().create(driveSubsystem));
        DriveWeightCommand.addWeight(new JoystickDriveWeight(driveController, gyro));
        configureBindings();
    }

    private void configureBindings() {
        driveController.start().onTrue(driveSubsystem.resetGyroCommand());
        driveController.leftBumper().onTrue(driveSubsystem.resetOdometryCommand(new Pose2d(0, 0, new Rotation2d(0))));
        // driveController.rightBumper().whileTrue(shooterSubsystem.setSpeedCommand(1, 1));
        driveController.b().onTrue(new InstantCommand(()->
            DriveWeightCommand.addWeight(new AutoDriveWeight(()->new Pose2d(), driveSubsystem::getPose, gyro))));
        driveController.b().onFalse(new InstantCommand(()->
            DriveWeightCommand.removeWeight("AutoDriveWeight")));
        //  driveController, gyro, new Pose2d(0,0,new Rotation2d(0))));
       
        driveController.rightTrigger().onTrue(new InstantCommand(()->
            DriveWeightCommand.addWeight(new MLVisionRotationDriveWeight(mlVision))));
        driveController.rightTrigger().onFalse(new InstantCommand(()->
            DriveWeightCommand.removeWeight("MLVisionRotationDriveWeight")));
    }

    public Command getAutonomousCommand() {
        return DashboardInit.getAutoChooserCommand();
    }
    public void removeAllDefaultCommands(){
        driveSubsystem.removeDefaultCommand();
        shooterSubsystem.removeDefaultCommand();
    }
}
