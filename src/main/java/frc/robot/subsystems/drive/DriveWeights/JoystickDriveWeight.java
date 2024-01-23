package frc.robot.subsystems.drive.DriveWeights;

import java.util.Arrays;
import java.util.NoSuchElementException;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.drive.JoystickCleaner;
import frc.lib.swerve.SwerveAlgorithms;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.SwerveDriveDimensions;
import frc.robot.sensors.Gyro.Gyro;
import frc.robot.subsystems.drive.DriveWeightCommand;

public class JoystickDriveWeight implements DriveWeight {
    // DRIVE COMMAND VARIABLES:
    private JoystickCleaner joystickCleaner = new JoystickCleaner();

    private final double SLOW_LINEAR_SPEED = 0.5;
    private final double SLOW_ROTATIONAL_SPEED = 0.3;

    private final double LOWER_DB = 0.15;
    private final double UPPER_DB = 0.15;

    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
    private boolean fieldRelative = true;

    private double iXSpeed;
    private double iYSpeed;
    private double angle;
    private double offset;

    double xSpeed;
    private double ySpeed;
    private double rot;

    private boolean hold = false;

    private Translation2d centerOfRot = new Translation2d();

    PIDController rotPID = PIDConstants.constructPID(PIDConstants.rotPID);;

    private double lastAngle;

    private CommandXboxController driverController;
    Gyro gyro;
    public JoystickDriveWeight(CommandXboxController driverController, Gyro gyro){
        this.driverController = driverController;
        this.gyro = gyro;
        lastAngle = gyro.getRawAngleRadians();
    }
    @Override
    public ChassisSpeeds getSpeeds() {
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

        /* Gyro correction */
        if (Math.abs(rot) == 0 && DriveWeightCommand.getWeights().size() == 1){
            rot = rotPID.calculate(SwerveAlgorithms.angleDistance(lastAngle, gyro.getRawAngleRadians()), 0);
            rot = MathUtil.clamp(rot, -1, 1);
        }
        else{
            lastAngle = gyro.getRawAngleRadians();
        }

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
            Logger.recordOutput("Drive/CoR", a);
            centerOfRot = a;
            return new ChassisSpeeds(xSpeed, ySpeed, rot);
            
        } else {
            centerOfRot = new Translation2d(0,0);
            return new ChassisSpeeds(xSpeed, ySpeed, rot);
            
        }
    }
    @Override
    public Translation2d getCenterOfRot() {
        return centerOfRot;
    }
}
