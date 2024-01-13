// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.proto.Kinematics;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveDriveDimensions;
import frc.robot.sensors.Gyro.Gyro;
import frc.robot.sensors.Gyro.GyroIO;
import frc.robot.sensors.Gyro.GyroIONavX;
import frc.robot.sensors.Gyro.GyroIOSim;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.commands.JoystickDriveCommand;
import frc.robot.subsystems.drive.commands.ResetGyro;

public class RobotContainer {
  private Gyro gyro;
  private DriveSubsystem driveSubsystem;
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final SendableChooser<Command> autoChooser;
  public RobotContainer() {
    
      switch (Robot.getMode()) {
        case REAL:
          gyro = new Gyro(new GyroIONavX());
          break;

        case SIM:
          gyro = new Gyro(new GyroIOSim(() -> Math.toDegrees(SwerveDriveDimensions.kinematics
            .toChassisSpeeds(
            driveSubsystem.getActualSwerveStates())
            .omegaRadiansPerSecond)));
          break;

        default:
          gyro = new Gyro(new GyroIO() {});
          break;
    }
    driveSubsystem = new DriveSubsystem(gyro);
    driveSubsystem.setDefaultCommand(new JoystickDriveCommand(driveSubsystem, gyro, driveController));

    //add pathplanner autochooser
    autoChooser = AutoBuilder.buildAutoChooser();
    ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    autoTab.add(autoChooser).withSize(5, 5).withPosition(1, 1);

    configureBindings();
  }

  private void configureBindings() {
    driveController.start().onTrue(new ResetGyro(driveSubsystem, gyro));
    driveController.leftBumper().onTrue(driveSubsystem.resetOdometryCommand(new Pose2d(0,0, new Rotation2d(0))));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
