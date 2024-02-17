package frc.robot.subsystems.drive.DriveWeights;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.swerve.SwerveAlgorithms;
import frc.robot.Constants.PIDConstants;
import frc.robot.sensors.Gyro.Gyro;

public class RotateLockWeight implements DriveWeight {
    Supplier<Pose2d> pose;
    Supplier<Pose2d> getPose;
    PIDController pidr = PIDConstants.constructPID(PIDConstants.rotPID);
    PIDController pidMoving = PIDConstants.constructPID(PIDConstants.rotMovingPID);
    Gyro gyro;
    Supplier<Double> getSpeed;
    double setAngle;

    public RotateLockWeight(Supplier<Pose2d> pose, Supplier<Pose2d> getPose, Gyro gyro, Supplier<Double> getSpeed) {
        this.pose = pose;
        this.getPose = getPose;
        this.gyro = gyro;
        this.getSpeed = getSpeed;
    }

    @Override
    public ChassisSpeeds getSpeeds() {
        double angle = Math.atan2(pose.get().getY() - getPose.get().getY(),
                pose.get().getX() - getPose.get().getX()) - gyro.getOffset();
        double o;
        
        if (getSpeed.get() > 0) {
            o = pidMoving.calculate(-SwerveAlgorithms.angleDistance(getPose.get().getRotation().getRadians(),
                    (angle + gyro.getOffset())), 0);
        } else {
            o = pidr.calculate(-SwerveAlgorithms.angleDistance(getPose.get().getRotation().getRadians(),
                    (angle + gyro.getOffset())), 0);
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
        setAngle = angle + gyro.getOffset();
        o = o * k;
        o = MathUtil.clamp(o, -1, 1);
        return new ChassisSpeeds(0, 0, o);
    }
    @Override
    public double angle(){
        return setAngle;
    }
}
