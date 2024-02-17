package frc.robot.subsystems.drive.DriveWeights;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.swerve.SwerveAlgorithms;
import frc.robot.Constants.PIDConstants;
import frc.robot.sensors.Gyro.Gyro;

public class AutoDriveWeight implements DriveWeight {
    Supplier<Pose2d> pose;
    Supplier<Pose2d> getPose;
    PIDController pid = PIDConstants.constructPID(PIDConstants.driveForwardPID);
    PIDController pidr = PIDConstants.constructPID(PIDConstants.rotPID);
    Gyro gyro;
    double setAngle;

    public AutoDriveWeight(Supplier<Pose2d> pose, Supplier<Pose2d> getPose, Gyro gyro) {
        this.pose = pose;
        this.getPose = getPose;
        this.gyro = gyro;
    }

    @Override
    public ChassisSpeeds getSpeeds() {
        double angle = Math.atan2(pose.get().getY() - getPose.get().getY(),
                pose.get().getX() - getPose.get().getX()) - gyro.getOffset();
        double dist = getPose.get().getTranslation().getDistance(pose.get().getTranslation());
        double s = pid.calculate(-dist, 0);
        double o = pidr.calculate(-SwerveAlgorithms.angleDistance(getPose.get().getRotation().getRadians(),
                pose.get().getRotation().getRadians()), 0);

        
        double scale = (dist / 3 + 1);
        s = MathUtil.clamp(s, -1, 1);
        o = MathUtil.clamp(o, -1, 1);
        if (Math.abs(o) < 0.01) {
            o = 0;
        }
        if (Math.abs(s) < 0.01) {
            s = 0;
        }
        setAngle = pose.get().getRotation().getRadians();
        // System.out.println(s);
        return new ChassisSpeeds(Math.sin(angle) * s / scale, -Math.cos(angle) * s / scale, o);
    }
    @Override
    public double angle(){
        return setAngle;
    }
}
