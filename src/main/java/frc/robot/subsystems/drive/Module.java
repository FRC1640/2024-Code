package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.PivotId;
import frc.robot.Constants.SwerveDriveDimensions;

public class Module{
    ModuleIO io;
    PivotId id;
    ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    public final PIDController drivePIDController = new PIDController(0.59818, 0.0, 0.0); 

	public final PIDController turningPIDController = new PIDController(0.725, 0.0, 0.005); //actual PID

    // public final PIDController turningPIDController = new PIDController(1,0,0); //sim PID

	public final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.10346, 2.9321, 0.11125); 

    public Module(ModuleIO io, PivotId id){
        this.io = io;
        this.id = id;
    }
    
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Modules/" + id, inputs);
    }
    public SwerveModuleState getState(){
        return new SwerveModuleState(inputs.driveVelocityMetersPerSecond, new Rotation2d(Math.toRadians(inputs.steerAngleDegrees)));
    }
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(inputs.drivePositionMeters, new Rotation2d(Math.toRadians(inputs.steerAngleDegrees)));
    }
    public double getVelocity(){
        return inputs.driveVelocityMetersPerSecond;
    }
    public void setDesiredState(SwerveModuleState state) { //TODO: clean up this method?
		double dAngle = state.angle.getDegrees() - inputs.steerAngleDegrees;
		double dAngleAbs = Math.abs(dAngle) % 360;
		boolean flipDriveTeleop = (90.0 <= dAngleAbs) && (dAngleAbs <= 270.0);
		double sin = Math.sin(Math.toRadians(dAngle));
		sin = (flipDriveTeleop) ? -sin : sin;
		double turnOutput = turningPIDController.calculate(sin, 0);

		final double targetSpeed = flipDriveTeleop ? state.speedMetersPerSecond : -state.speedMetersPerSecond;

		if (Math.abs(targetSpeed) / SwerveDriveDimensions.maxSpeed < 0.1) {
			turnOutput = 0;
		}

		io.setDrivePercentage(targetSpeed / SwerveDriveDimensions.maxSpeed);
		io.setSteerPercentage(turnOutput);
	}
}
