package frc.lib.drive.Module;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.drive.SwerveAlgorithms;
import frc.robot.Robot;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.PivotId;
import frc.robot.Constants.SwerveDriveDimensions;

public class Module {
    ModuleIO io;
    PivotId id;
    ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    public final PIDController drivePIDController = PIDConstants.constructPID(PIDConstants.drivePIDController,
            "module drive");

    public final PIDController turningPIDController = PIDConstants.constructPID(PIDConstants.turningPIDController,
            "module turning");

    // public final PIDController turningPIDController = new PIDController(1,0,0);
    // //sim PID

    public final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.21414, 2.5109, 0.25135);

    private double pidModuleTarget;
    SlewRateLimiter voltageLimiter = new SlewRateLimiter(120);

    public Module(ModuleIO io, PivotId id) {
        this.io = io;
        this.id = id;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Modules/" + id, inputs);
        // Logger.recordOutput("Drive/Modules/" + id, );
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

    public boolean getError(){
        Logger.recordOutput("angle delta", SwerveAlgorithms.angleDistance(pidModuleTarget,inputs.steerAngleAbsolute));
        return Math.abs(SwerveAlgorithms.angleDistance(pidModuleTarget,inputs.steerAngleAbsolute)) < Math.toRadians(5)
        || Math.abs(SwerveAlgorithms.angleDistance(pidModuleTarget + Math.PI,inputs.steerAngleAbsolute)) < Math.toRadians(5);
    }

    public void pidModule(double angle) {
        double dAngle = SwerveAlgorithms.angleDistance(inputs.steerAngleAbsolute, angle);

        // determines if drive should be flipped so max delta angle is 90 degrees
        boolean flipDriveTeleop = (Math.PI / 2 <= Math.abs(dAngle)) && (Math.abs(dAngle) <= 3 * Math.PI / 2);

        

        // pid calculation
        double sin = Math.sin(dAngle);
        sin = (flipDriveTeleop) ? -sin : sin;
        double turnOutput = turningPIDController.calculate(sin, 0);
        io.setSteerPercentage(turnOutput);

        pidModuleTarget = angle;

    }

    public void setDesiredStateMetersPerSecond(SwerveModuleState state) {
        Rotation2d delta = state.angle.minus(new Rotation2d(inputs.steerAngleRadians));

        // double dAngle = SwerveAlgorithms.angleDistance(inputs.steerAngleRadians, state.angle.getRadians());
        boolean flipDriveTeleop = false;
        if (Math.abs(delta.getDegrees()) > 90.0) {
            flipDriveTeleop = true;
        }
        // determines if drive should be flipped so max delta angle is 90 degrees
        

        // pid calculation
        double sin = Math.sin(delta.getRadians());
        sin = (flipDriveTeleop) ? -sin : sin;
        double turnOutput = turningPIDController.calculate(sin, 0);

        // flips drive
        final double targetSpeed = (flipDriveTeleop ? state.speedMetersPerSecond : -state.speedMetersPerSecond) * Math.abs(Math.cos(delta.getRadians()));

        // calculates drive speed with feedforward
        double pidSpeed = (driveFeedforward.calculate(targetSpeed));
        pidSpeed += drivePIDController.calculate(inputs.driveVelocityMetersPerSecond, targetSpeed); // feedforward calc

        // double pidSpeed = targetSpeed / SwerveDriveDimensions.maxSpeed * 12;
        // if (!Robot.inTeleop) {
        //     pidSpeed += drivePIDController.calculate(inputs.driveVelocityMetersPerSecond, targetSpeed);
        // }
        

        // pid clamping and deadband
        pidSpeed = MathUtil.clamp(pidSpeed, -12, 12);

        

        if (Math.abs(pidSpeed) < 0.1) {
            turnOutput = 0;
        }

        io.setDriveVoltage(voltageLimiter.calculate(pidSpeed));
        Logger.recordOutput("Drive/Modules/" + id + "NonLimitedSpeed", pidSpeed);
        Logger.recordOutput("Drive/Modules/" + id + "LimitedSpeed", voltageLimiter.calculate(pidSpeed));
        io.setSteerPercentage(turnOutput);
    }

    public void setDriveVoltage(double volts) {
        io.setDriveVoltage(voltageLimiter.calculate(volts));
        // io.setDriveVoltage(volts);
    }

    public double getDriveVoltage() {
        return inputs.driveAppliedVoltage;
    }

    public void setBrakeMode(boolean brake) {
        io.setBrakeMode(brake);
    }

    public SwerveModulePosition[] getOdometryPositions() {
        int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        SwerveModulePosition[] odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositionsMeters[i];
            Rotation2d angle = inputs.odometryTurnPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }
        return odometryPositions;
    }

    public void resetSteer() {
        io.resetSteer();
    }

    /** Returns the timestamps of the samples received this cycle. */
    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }
}
