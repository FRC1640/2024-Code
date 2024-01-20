// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveDriveDimensions;
import frc.robot.Robot.TestMode;
import frc.robot.sensors.Gyro.Gyro;
import frc.robot.sensors.Gyro.GyroIO;
import frc.robot.sensors.Gyro.GyroIONavX;
import frc.robot.sensors.Gyro.GyroIOSim;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.commands.JoystickDriveCommand;
import frc.robot.subsystems.drive.commands.ResetGyro;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RobotContainer {
    private Gyro gyro;
    private DriveSubsystem driveSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private final CommandXboxController driveController = new CommandXboxController(0);

    public RobotContainer() {
        // Dashboard init
        
        switch (Robot.getMode()) {
            case REAL:
                gyro = new Gyro(new GyroIONavX());
                shooterSubsystem = new ShooterSubsystem(new ShooterIOSparkMax());
                break;

            case SIM:
                gyro = new Gyro(new GyroIOSim(() -> Math.toDegrees(SwerveDriveDimensions.kinematics
                        .toChassisSpeeds(
                                driveSubsystem.getActualSwerveStates()).omegaRadiansPerSecond)));
                shooterSubsystem = new ShooterSubsystem(new ShooterIO(){});
                break;

            default:
                gyro = new Gyro(new GyroIO(){});
                shooterSubsystem = new ShooterSubsystem(new ShooterIO(){});
                break;
        }
        driveSubsystem = new DriveSubsystem(gyro);
        
        DashboardInit.init(driveSubsystem, driveController);
        if (DashboardInit.getTestMode() != TestMode.SYSID){
            driveSubsystem.setDefaultCommand(new JoystickDriveCommand(driveSubsystem, gyro, driveController));
            shooterSubsystem.setDefaultCommand(shooterSubsystem.setSpeedCommand(0.5, 0.5));
            configureBindings();
        }
    }

    private void configureBindings() {
        driveController.start().onTrue(new ResetGyro(driveSubsystem, gyro));
        driveController.leftBumper().onTrue(driveSubsystem.resetOdometryCommand(new Pose2d(0, 0, new Rotation2d(0))));
        driveController.rightBumper().whileTrue(shooterSubsystem.setSpeedCommand(1, 1));
    }

    public Command getAutonomousCommand() {
        return DashboardInit.getAutoChooserCommand();
    }
}
