// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.drive.DriveSubsystem;
import frc.robot.Constants.SwerveDriveDimensions;
import frc.robot.Robot.TestMode;
import frc.robot.sensors.Gyro.Gyro;
import frc.robot.sensors.Gyro.GyroIO;
import frc.robot.sensors.Gyro.GyroIONavX;
import frc.robot.sensors.Gyro.GyroIOSim;
import frc.robot.sensors.Vision.AprilTagVision;
import frc.robot.sensors.Vision.AprilTagVisionIO;
import frc.robot.sensors.Vision.AprilTagVisionIOLimelight;
import frc.robot.sensors.Vision.AprilTagVisionIOSim;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.drive.JoystickDriveCommand;

public class RobotContainer {

  private Gyro gyro;
  private AprilTagVision aprilTagVision;
  private DriveSubsystem driveSubsystem;
  private final CommandXboxController driveController = new CommandXboxController(0);
  private ShooterSubsystem shooterSubsystem;
  public RobotContainer() {
      switch (Robot.getMode()) {
        case REAL:
          gyro = new Gyro(new GyroIONavX());
          aprilTagVision = new AprilTagVision(new AprilTagVisionIOLimelight());
          shooterSubsystem = new ShooterSubsystem(new ShooterIOSparkMax());
          break;
        case SIM:
                gyro = new Gyro(new GyroIOSim(() -> Math.toDegrees(SwerveDriveDimensions.kinematics
                        .toChassisSpeeds(
                                driveSubsystem.getActualSwerveStates()).omegaRadiansPerSecond)));
                shooterSubsystem = new ShooterSubsystem(new ShooterIO(){});
                aprilTagVision = new AprilTagVision(new AprilTagVisionIOSim());
                break;

         default:
                gyro = new Gyro(new GyroIO(){});
                shooterSubsystem = new ShooterSubsystem(new ShooterIO(){});
                aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {});
                break;
        }
        driveSubsystem = new DriveSubsystem(gyro, aprilTagVision);
        DashboardInit.init(driveSubsystem, driveController);
        if (DashboardInit.getTestMode() != TestMode.SYSID){
            shooterSubsystem.setDefaultCommand(shooterSubsystem.setSpeedCommand(0.5, 0.5));
            driveSubsystem.setDefaultCommand(new JoystickDriveCommand().create(driveSubsystem, driveController, gyro));
            configureBindings();
        }
    }

    private void configureBindings() {
        driveController.start().onTrue(driveSubsystem.resetGyroCommand());
        driveController.leftBumper().onTrue(driveSubsystem.resetOdometryCommand(new Pose2d(0, 0, new Rotation2d(0))));
        // driveController.rightBumper().whileTrue(shooterSubsystem.setSpeedCommand(1, 1));
        driveController.rightBumper().whileTrue(new JoystickDriveCommand().create(driveSubsystem,
         driveController, gyro, new Pose2d(0,0,new Rotation2d(0))));
    }

    public Command getAutonomousCommand() {
        return DashboardInit.getAutoChooserCommand();
    }
}
