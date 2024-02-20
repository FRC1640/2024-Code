package frc.robot.subsystems.drive.DriveWeights;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.swerve.SwerveAlgorithms;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.drive.DriveWeightCommand;

public class RotateToAngleWeight implements DriveWeight {
    PIDController pidr;
    PIDController pidMoving;
    Supplier<Double> getSpeed;
    Supplier<Pose2d> getPose;
    DoubleSupplier angle;
    private BooleanSupplier cancelCondition;

    public RotateToAngleWeight(DoubleSupplier angle, Supplier<Pose2d> getPose, Supplier<Double> getSpeed, String name,
            BooleanSupplier cancelCondition) {
        this.angle = angle;
        this.getPose = getPose;
        this.getSpeed = getSpeed;
        this.cancelCondition = cancelCondition;
        pidr = PIDConstants.constructPID(PIDConstants.rotPID, "RotateToAngle" + name);
        pidMoving = PIDConstants.constructPID(PIDConstants.rotMovingPID, "RotateToAngleMoving" + name);
    }

    @Override
    public ChassisSpeeds getSpeeds() {
        if (cancelCondition.getAsBoolean()){
            DriveWeightCommand.removeWeight(this);
        }
        double o;
        
        if (getSpeed.get() > 0) {
            o = pidMoving.calculate(-SwerveAlgorithms.angleDistance(getPose.get().getRotation().getRadians(),
                    angle.getAsDouble()), 0);
        } else {
            o = pidr.calculate(-SwerveAlgorithms.angleDistance(getPose.get().getRotation().getRadians(),
                    angle.getAsDouble()), 0);
        }

        if (Math.abs(o) < 0.005) {
            o = 0;
        }
        
        double k;
        double linearRotSpeed = Math.abs(o * SwerveAlgorithms.maxNorm);
        o = MathUtil.clamp(o, -1, 1);
        if (getSpeed.get() == 0 || o ==0){
            k = 1;
        }
        else{
            k = 1+Math.min(getSpeed.get() / linearRotSpeed, linearRotSpeed / getSpeed.get());
        }
        o = o * k;
        o = MathUtil.clamp(o, -1, 1);
        return new ChassisSpeeds(0, 0, o);
    }
    @Override
    public double angle(){
        return angle.getAsDouble();
    }
}
