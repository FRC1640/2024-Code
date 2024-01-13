package frc.robot.subsystems.drive.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.swerve.SwerveAlgorithms;
import frc.robot.Constants.SwerveDriveDimensions;
import frc.robot.sensors.Gyro.Gyro;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.JoystickCleaner;

import java.util.Arrays;
import java.util.NoSuchElementException;
import java.util.OptionalDouble;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;

public class JoystickDriveCommand extends Command {
    final double SLOW_LINEAR_SPEED = 0.6;
    final double SLOW_ROTATIONAL_SPEED = 0.55;

    final double LOWER_DB = 0.15;
    final double UPPER_DB = 0.15;

    DriveSubsystem driveSubsystem;
    Gyro gyro;
    CommandXboxController driverController;

    JoystickCleaner joystickCleaner = new JoystickCleaner();

    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
    private boolean fieldRelative = true;

    double iXSpeed;
    double iYSpeed;
    double angle;
    double offset;

    double xSpeed;
    double ySpeed;
    double rot;

    boolean hold = false;

    public JoystickDriveCommand(DriveSubsystem driveSubsystem, Gyro gyro, CommandXboxController driveController) {
        this.driveSubsystem = driveSubsystem;
        this.gyro = gyro;
        this.driverController = driveController;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {


        driverController.back().onTrue(new InstantCommand(() -> fieldRelative = !fieldRelative));

        Trigger leftTrigger = new Trigger(() -> driverController.getLeftTriggerAxis() > 0.1);

        Trigger rightTrigger = new Trigger(() -> driverController.getRightTriggerAxis() > 0.1);

        if(!rightTrigger.getAsBoolean()) {
            xSpeed = -driverController.getLeftY() * SLOW_LINEAR_SPEED;
            ySpeed = -driverController.getLeftX() * SLOW_LINEAR_SPEED;
            rot = -driverController.getRightX() * SLOW_ROTATIONAL_SPEED;
        } 
        else {
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
    
        if (!hold && leftTrigger.getAsBoolean()){
            iXSpeed = xSpeed;
            iYSpeed = ySpeed;
            offset = -gyro.getAngleRotation2d().getRadians();
            hold = true;
        }
        if (!leftTrigger.getAsBoolean()){
            hold = false;
        }
        angle = (Math.atan2(iYSpeed, iXSpeed) + offset); //add gyro offset to angle
        if (leftTrigger.getAsBoolean()){ //drive with center of rot around closest pivot
            Translation2d a = Arrays.stream(SwerveDriveDimensions.positions)
                .reduce((best, current) ->
                Math.abs((SwerveAlgorithms.angleDistance(Math.atan2(current.getY(), current.getX()), angle))) < 
                Math.abs((SwerveAlgorithms.angleDistance(Math.atan2(best.getY(), best.getX()), angle)))
                ? current:best)
                .orElseThrow(() -> new NoSuchElementException("No closest pivot."));
            driveSubsystem.drivePercentDoubleCone(xSpeed, ySpeed, rot, fieldRelative, a);
            Logger.recordOutput("Drive/CoR",a);
        }
        else{
            driveSubsystem.drivePercentDoubleCone(xSpeed, ySpeed, rot, fieldRelative);
        }
        
    }
    @Override
    public void end(boolean interrupted) { }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}