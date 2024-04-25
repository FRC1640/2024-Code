package frc.robot.subsystems.drive.DriveWeights;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.drive.DriveWeight;
import frc.lib.drive.SwerveAlgorithms;
import frc.robot.Constants.PIDConstants;
import frc.robot.sensors.Gyro.Gyro;

public class RotateLockWeightTrack implements DriveWeight {
    Supplier<Pose2d> pose;
    Supplier<Pose2d> getPose;
    PIDController pidr = PIDConstants.constructPID(PIDConstants.rotPID, "rotlock");
    PIDController pidMoving = PIDConstants.constructPID(PIDConstants.rotMovingPID, "rotlockmoving");
    PIDController pidTarget = PIDConstants.constructPID(PIDConstants.rotToSpeaker, "rottospeaker");
    Gyro gyro;
    Supplier<Double> getSpeed;
    double setAngle;
    private DoubleSupplier track;
    private BooleanSupplier canSee;

    public RotateLockWeightTrack(Supplier<Pose2d> pose, Supplier<Pose2d> getPose, Gyro gyro, Supplier<Double> getSpeed,
            DoubleSupplier track,
            BooleanSupplier canSee) {
        this.pose = pose;
        this.getPose = getPose;
        this.gyro = gyro;
        this.getSpeed = getSpeed;
        this.track = track;
        this.canSee = canSee;
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
        if (getSpeed.get() == 0 || o == 0) {
            k = 1;
        } else {
            k = 1 + Math.min(getSpeed.get() / linearRotSpeed, linearRotSpeed / getSpeed.get());
        }
        setAngle = angle + gyro.getOffset();
        o = o * k;
        o = MathUtil.clamp(o, -1, 1);

        if (canSee.getAsBoolean()) {
            double s1 = pidTarget.calculate(track.getAsDouble(), 0);
            return new ChassisSpeeds(0, 0, s1);
        } else {
            
            return new ChassisSpeeds(0, 0, o);
        }

    }

    @Override
    public double angle() {
        return setAngle;
    }
}