// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.sensors.Gyro.Gyro;
import frc.robot.sensors.Gyro.GyroIO;
import frc.robot.sensors.Gyro.GyroIONavX;
import frc.robot.subsystems.drive.DriveSubsystem;

public class RobotContainer {
  private Gyro gyro;
  private DriveSubsystem driveSubsystem;
  public RobotContainer() {
    
      switch (Robot.getMode()) {
        case REAL:
          gyro = new Gyro(new GyroIONavX());
          break;

        case SIM:
          gyro = new Gyro(new GyroIO() {});
          break;

        default:
          gyro = new Gyro(new GyroIO() {});
          break;
    }
    driveSubsystem = new DriveSubsystem(gyro);
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
