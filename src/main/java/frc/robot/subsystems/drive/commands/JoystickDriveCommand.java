package frc.robot.subsystems.drive.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.sensors.Gyro.Gyro;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.JoystickCleaner;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class JoystickDriveCommand extends Command {
    final double SLOW_LINEAR_SPEED = 0.6;
    final double SLOW_ROTATIONAL_SPEED = 0.55;

    final double SUPER_SLOW_LINEAR_SPEED = 0.35; // 58.3%
    final double SUPER_SLOW_ROTATIONAL_SPEED = 0.32;

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
        double xSpeed;
        double ySpeed;
        double rot;

        driverController.back().onTrue(new InstantCommand(() -> fieldRelative = !fieldRelative));

        Trigger leftTrigger = new Trigger(() -> driverController.getLeftTriggerAxis() > 0.1);

        Trigger rightTrigger = new Trigger(() -> driverController.getRightTriggerAxis() > 0.1);

        if(!leftTrigger.getAsBoolean()) {
            xSpeed = -driverController.getLeftY() * SLOW_LINEAR_SPEED;
            ySpeed = -driverController.getLeftX() * SLOW_LINEAR_SPEED;
            rot = -driverController.getRightX() * SLOW_ROTATIONAL_SPEED;
        } 
        else {
            xSpeed = -m_xspeedLimiter.calculate(driverController.getLeftY());
            ySpeed = -m_yspeedLimiter.calculate(driverController.getLeftX());
            rot = -m_rotLimiter.calculate(driverController.getRightX());
        }
        if(rightTrigger.getAsBoolean()){
            xSpeed = -driverController.getLeftY() * SUPER_SLOW_LINEAR_SPEED;
            ySpeed = -driverController.getLeftX() * SUPER_SLOW_LINEAR_SPEED;
            rot = -driverController.getRightX() * SUPER_SLOW_ROTATIONAL_SPEED;
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
        driveSubsystem.drivePercentDesaturated(xSpeed, ySpeed, rot, fieldRelative);
    }
    @Override
    public void end(boolean interrupted) { }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}