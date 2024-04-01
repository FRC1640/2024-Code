package frc.robot.subsystems.drive.DriveWeights;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.swerve.SwerveAlgorithms;
import frc.robot.Constants.PIDConstants;
import frc.robot.sensors.Gyro.Gyro;

public class AutoDriveWeight implements DriveWeight {
    Supplier<Pose2d> pose;
    Supplier<Pose2d> getPose;
    PIDController pid = PIDConstants.constructPID(PIDConstants.driveForwardPID, "autodriveforward");
    PIDController pidr = PIDConstants.constructPID(PIDConstants.rotPID, "autodriverotation");
    Gyro gyro;
    double setAngle;

    public AutoDriveWeight(Supplier<Pose2d> destinationPose, Supplier<Pose2d> currentPose, Gyro gyro) {
        this.pose = destinationPose;
        this.getPose = currentPose;
        this.gyro = gyro;
    }

    @Override
    public ChassisSpeeds getSpeeds() {
        double angle = new Rotation2d(pose.get().getX() - getPose.get().getX(), pose.get().getY() - getPose.get().getY()).getRadians();
        double dist = getPose.get().getTranslation().getDistance(pose.get().getTranslation());
        double s = pid.calculate(dist, 0);
        double o = pidr.calculate(-SwerveAlgorithms.angleDistance(getPose.get().getRotation().getRadians(),
                pose.get().getRotation().getRadians()), 0);

        //double scale = (dist / 3 + 1);
        double scale = 1;
        s = Math.abs(s);
        s = MathUtil.clamp(s, -1, 1);
        o = MathUtil.clamp(o, -1, 1);
        if (Math.abs(o) < 0.01) {
            o = 0;
        }
        if (Math.abs(s) < 0.01) {
            s = 0;
        }
        setAngle = pose.get().getRotation().getRadians();
        Logger.recordOutput("AutoDriveAngle", Math.toDegrees(angle));
        Logger.recordOutput("AutoDrivePose", getPose.get());
        Logger.recordOutput("AutoDriveDestination", pose.get());
        double xSpeed = (Math.cos(angle) * s / scale);
        double ySpeed = (Math.sin(angle) * s / scale);
        double offset = gyro.getOffset();
        ChassisSpeeds cspeeds = new ChassisSpeeds(xSpeed*Math.cos(offset)+ySpeed*Math.sin(offset), ySpeed*Math.cos(offset)-xSpeed*Math.sin(offset), o);
        return cspeeds;
    }
    @Override
    public double angle(){
        return setAngle;
    }
}