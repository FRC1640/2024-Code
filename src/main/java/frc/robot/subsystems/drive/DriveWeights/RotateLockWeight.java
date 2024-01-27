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
    Gyro gyro;

    public RotateLockWeight(Supplier<Pose2d> pose, Supplier<Pose2d> getPose, Gyro gyro) {
        this.pose = pose;
        this.getPose = getPose;
        this.gyro = gyro;
    }

    @Override
    public ChassisSpeeds getSpeeds() {
        double angle = Math.atan2(pose.get().getY() - getPose.get().getY(),
                pose.get().getX() - getPose.get().getX()) - gyro.getOffset();
        double o = pidr.calculate(-SwerveAlgorithms.angleDistance(getPose.get().getRotation().getRadians(),
                (angle + gyro.getOffset())), 0);
        if (Math.abs(o) < 0.01) {
            o = 0;
        }
        double scale = 1;
        o = MathUtil.clamp(o, -1, 1);
        return new ChassisSpeeds(0, 0, o / scale);
    }
}
