package frc.robot.subsystems.drive.DriveWeights;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.swerve.SwerveAlgorithms;
import frc.robot.Constants.PIDConstants;

public class RotateToAngleWeight implements DriveWeight {
    Supplier<Pose2d> pose;
    PIDController pidr = PIDConstants.constructPID(PIDConstants.rotPID);
    PIDController pidMoving = PIDConstants.constructPID(PIDConstants.rotMovingPID);
    Supplier<Double> getSpeed;
    Supplier<Pose2d> getPose;
    double angle;

    public RotateToAngleWeight(Supplier<Pose2d> pose, double angle, Supplier<Pose2d> getPose, Supplier<Double> getSpeed) {
        this.pose = pose;
        this.angle = angle;
        this.getPose = getPose;
        this.getSpeed = getSpeed;
    }

    @Override
    public ChassisSpeeds getSpeeds() {
        double o;
        
        if (getSpeed.get() > 0) {
            o = pidMoving.calculate(-SwerveAlgorithms.angleDistance(getPose.get().getRotation().getRadians(),
                    angle), 0);
        } else {
            o = pidr.calculate(-SwerveAlgorithms.angleDistance(getPose.get().getRotation().getRadians(),
                    angle), 0);
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
        return angle;
    }
}
