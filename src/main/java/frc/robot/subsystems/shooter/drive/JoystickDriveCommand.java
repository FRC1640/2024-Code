package frc.robot.subsystems.shooter.drive;

import java.util.Arrays;
import java.util.NoSuchElementException;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.drive.DriveSubsystem;
import frc.lib.drive.JoystickCleaner;
import frc.lib.swerve.SwerveAlgorithms;
import frc.robot.Constants.SwerveDriveDimensions;
import frc.robot.sensors.Gyro.Gyro;

public class JoystickDriveCommand {
    // DRIVE COMMAND VARIABLES:
    JoystickCleaner joystickCleaner = new JoystickCleaner();

    final double SLOW_LINEAR_SPEED = 0.5;
    final double SLOW_ROTATIONAL_SPEED = 0.3;

    Translation2d centerOfRot;

    PIDController pid = new PIDController(1,0,0);

    PIDController pidr = new PIDController(1,0,0);

    final double LOWER_DB = 0.15;
    final double UPPER_DB = 0.15;

    final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
    boolean fieldRelative = true;

    double iXSpeed;
    double iYSpeed;
    double angle;
    double offset;

    double xSpeed;
    double ySpeed;
    double rot;

    boolean hold = false;

    ChassisSpeeds speeds= new ChassisSpeeds();

    public Command create(DriveSubsystem driveSubsystem, CommandXboxController driverController, Gyro gyro, Pose2d pose) {
        Command c = Commands.race(
                driverController.rightBumper().getAsBoolean()?
                new RunCommand(() -> setChassisSpeeds(driverController, gyro, pose, driveSubsystem))
                :new RunCommand(() -> setChassisSpeeds(driverController, gyro), new Subsystem[]{}),
                driveSubsystem.driveDoubleConeCommand(() -> speeds, () -> centerOfRot)
                .andThen(driveSubsystem.driveDoubleConeCommand(() -> 
                new ChassisSpeeds(0,0,0), () -> new Translation2d(0,0))));
        c.addRequirements(driveSubsystem);
        return c;
    }

    public void setChassisSpeeds(CommandXboxController driverController, Gyro gyro, Pose2d pose, DriveSubsystem driveSubsystem){
        setChassisSpeeds(driverController, gyro);
        double angle = Math.atan2(pose.getY() - driveSubsystem.getPose().getY(), pose.getX() - driveSubsystem.getPose().getX());
        double dist = driveSubsystem.getPose().getTranslation().getDistance(pose.getTranslation());
        double s = pid.calculate(dist, 0);
        double o = pidr.calculate(driveSubsystem.getPose().getRotation().getRadians(), pose.getRotation().getRadians());
        speeds.plus(new ChassisSpeeds(Math.cos(angle) * s / dist,Math.sin(angle) * s / dist,o / dist));
    }

    public void setChassisSpeeds(CommandXboxController driverController, Gyro gyro) {
        driverController.back().onTrue(new InstantCommand(() -> fieldRelative = !fieldRelative));

        Trigger leftTrigger = new Trigger(() -> driverController.getLeftTriggerAxis() > 0.1);

        Trigger rightTrigger = new Trigger(() -> driverController.getRightTriggerAxis() > 0.1);

        if (!rightTrigger.getAsBoolean()) {
            xSpeed = -driverController.getLeftY() * SLOW_LINEAR_SPEED;
            ySpeed = -driverController.getLeftX() * SLOW_LINEAR_SPEED;
            rot = -driverController.getRightX() * SLOW_ROTATIONAL_SPEED;
        } else {
            xSpeed = -m_xspeedLimiter.calculate(driverController.getLeftY());
            ySpeed = -m_yspeedLimiter.calculate(driverController.getLeftX());
            rot = -m_rotLimiter.calculate(driverController.getRightX());
        }

        /* Apply linear deadband */
        joystickCleaner.setX(xSpeed);
        joystickCleaner.setY(ySpeed);
        joystickCleaner.applyDeadband(LOWER_DB, UPPER_DB);
        xSpeed = joystickCleaner.getX();
        ySpeed = joystickCleaner.getY();

        /* Apply rotational deadband */
        rot = MathUtil.applyDeadband(rot, LOWER_DB);

        /* Increase rotational sensitivity */
        rot = Math.signum(rot) * Math.pow(Math.abs(rot), 1.0 / 3.0);

        if (!hold && leftTrigger.getAsBoolean()) {
            iXSpeed = xSpeed;
            iYSpeed = ySpeed;
            offset = -gyro.getAngleRotation2d().getRadians();
            hold = true;
        }
        if (!leftTrigger.getAsBoolean()) {
            hold = false;
        }
        angle = (Math.atan2(iYSpeed, iXSpeed) + offset); // add gyro offset to angle
        if (leftTrigger.getAsBoolean()) { // drive with center of rot around closest pivot
            Translation2d a = Arrays.stream(SwerveDriveDimensions.positions)
                    .reduce((best,
                            current) -> Math.abs((SwerveAlgorithms
                                    .angleDistance(Math.atan2(current.getY(), current.getX()), angle))) < Math
                                            .abs((SwerveAlgorithms.angleDistance(
                                                    Math.atan2(best.getY(), best.getX()), angle)))
                                                            ? current
                                                            : best)
                    .orElseThrow(() -> new NoSuchElementException("No closest pivot."));
            centerOfRot = a;
            speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
            Logger.recordOutput("Drive/CoR", a);
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
            centerOfRot = new Translation2d(0,0);
        }
    }
}
