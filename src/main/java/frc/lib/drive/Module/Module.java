package frc.lib.drive.Module;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.swerve.SwerveAlgorithms;
import frc.robot.Constants.PivotId;
import frc.robot.Constants.SwerveDriveDimensions;

public class Module {
    ModuleIO io;
    PivotId id;
    ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    public final PIDController drivePIDController = new PIDController(0, 0.0, 0);

    public final PIDController turningPIDController = new PIDController(0.725, 0.0, 0.005); // actual PID

    // public final PIDController turningPIDController = new PIDController(1,0,0);
    // //sim PID

    public final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(-0.049744, 2.8423, 0.13785);

    public Module(ModuleIO io, PivotId id) {
        this.io = io;
        this.id = id;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Modules/" + id, inputs);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(inputs.driveVelocityMetersPerSecond,
                new Rotation2d(Math.toRadians(inputs.steerAngleDegrees)));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(inputs.drivePositionMeters,
                new Rotation2d(Math.toRadians(inputs.steerAngleDegrees)));
    }

    public double getVelocity() {
        return inputs.driveVelocityMetersPerSecond;
    }

    public void setDesiredStateMetersPerSecond(SwerveModuleState state) {
        double dAngle = SwerveAlgorithms.angleDistance(inputs.steerAngleRadians, state.angle.getRadians()); //gets angle delta

        // determines if drive should be flipped so max delta angle is 90 degrees
        boolean flipDriveTeleop = (Math.PI / 2 <= Math.abs(dAngle)) && (Math.abs(dAngle) <= 3 * Math.PI / 2); 

        // pid calculation
        double sin = Math.sin(dAngle);
        sin = (flipDriveTeleop) ? -sin : sin;
        double turnOutput = turningPIDController.calculate(sin, 0);

        // flips drive
        final double targetSpeed = flipDriveTeleop ? state.speedMetersPerSecond : -state.speedMetersPerSecond;

        //calculates drive speed with feedforward
        double pidSpeed = (driveFeedforward.calculate(targetSpeed) + 
            drivePIDController.calculate(inputs.driveVelocityMetersPerSecond, targetSpeed)); 

        //pid clamping and deadband
        pidSpeed = MathUtil.clamp(pidSpeed, -12, 12);

        if (Math.abs(pidSpeed) < 0.05) {
            turnOutput = 0;
        }

        io.setDriveVoltage(pidSpeed);
        io.setSteerPercentage(turnOutput);
    }

    public void setDriveVoltage(double volts) {
        io.setDriveVoltage(volts);
    }

    public double getDriveVoltage() {
        return inputs.driveAppliedVoltage;
    }
}
