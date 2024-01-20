// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.commands.JoystickDriveCommand;
import frc.robot.subsystems.drive.commands.ResetGyro;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RobotContainer {
  private Gyro gyro;
  private AprilTagVision aprilTagVision;
  private DriveSubsystem driveSubsystem;
  private final CommandXboxController driveController = new CommandXboxController(0);
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
        DashboardInit.init(driveSubsystem, driveController);
        if (DashboardInit.getTestMode() != TestMode.SYSID){
            driveSubsystem.setDefaultCommand(new JoystickDriveCommand(driveSubsystem, gyro, driveController));
            configureBindings();
        }
    }

    private void configureBindings() {
        driveController.start().onTrue(new ResetGyro(driveSubsystem, gyro));
        driveController.leftBumper().onTrue(driveSubsystem.resetOdometryCommand(new Pose2d(0, 0, new Rotation2d(0))));
        new Trigger(() -> intakeSubsystem.getHasNote()).whileTrue(intakeSubsystem.intakeCommand(1.0));
    }

    public Command getAutonomousCommand() {
        return DashboardInit.getAutoChooserCommand();
    }
}