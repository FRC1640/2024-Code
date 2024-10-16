package frc.lib.drive;

import java.util.Arrays;
import java.util.NoSuchElementException;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveDriveDimensions;

public class SwerveAlgorithms {
    public static double maxNorm = computeMaxNorm(SwerveDriveDimensions.positions, new Translation2d(0, 0));

    public static SwerveModuleState[] desaturated(double xSpeed, double ySpeed, double rot, double currentAngleRadians,
            boolean fieldRelative) {
        var swerveModuleStates = SwerveDriveDimensions.kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
                                new Rotation2d(currentAngleRadians))
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveDriveDimensions.maxSpeed);
        return swerveModuleStates;
    }

    public static SwerveModuleState[] rawSpeeds(double xSpeed, double ySpeed, double rot) {
        var swerveModuleStates = SwerveDriveDimensions.kinematics
                .toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
        return swerveModuleStates;
    }

    public static SwerveModuleState[] desaturated(double xSpeed, double ySpeed, double rot,
            double currentAngleRadians, boolean fieldRelative, Translation2d centerOfRotation) {

        var swerveModuleStates = SwerveDriveDimensions.kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
                                new Rotation2d(currentAngleRadians))
                        : new ChassisSpeeds(xSpeed, ySpeed, rot),
                centerOfRotation);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveDriveDimensions.maxSpeed);
        return swerveModuleStates;
    }

    public static SwerveModuleState[] doubleCone(double xSpeed, double ySpeed, double rot,
            double currentAngleRadians, boolean fieldRelative) {

        double translationalSpeed = Math.hypot(xSpeed, ySpeed);
        double linearRotSpeed = Math.abs(rot * maxNorm);
        double k;
        if (linearRotSpeed == 0 || translationalSpeed == 0) {
            k = 0;
        } else {
            k = Math.max(linearRotSpeed, translationalSpeed) / (linearRotSpeed + translationalSpeed);
        }
        var swerveModuleStates = SwerveDriveDimensions.kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(k * xSpeed, k * ySpeed, k * rot,
                                new Rotation2d(currentAngleRadians))
                        : new ChassisSpeeds(xSpeed * k, ySpeed * k, rot * k));
        return swerveModuleStates;
    }

    public static SwerveModuleState[] doubleCone(double xSpeed, double ySpeed, double rot,
            double currentAngleRadians, boolean fieldRelative, Translation2d centerOfRotation, boolean lockRotation) {

        double translationalSpeed = Math.hypot(xSpeed, ySpeed);

        double linearRotSpeed = Math.abs(rot * computeMaxNorm(SwerveDriveDimensions.positions, centerOfRotation));
        double k;

        // determine scaling factor for double cone map
        if (linearRotSpeed == 0 || translationalSpeed == 0) {
            k = 0;
        } else {
            k = Math.min(translationalSpeed / linearRotSpeed, linearRotSpeed / translationalSpeed);
        }
        if (lockRotation) {
            double scale = Math.abs((1 - Math.abs(linearRotSpeed) / (SwerveDriveDimensions.maxSpeed)));
            Logger.recordOutput("ScaleDriveWeight", scale);
            var swerveModuleStates = SwerveDriveDimensions.kinematics.toSwerveModuleStates(
                    fieldRelative
                            ? ChassisSpeeds.fromFieldRelativeSpeeds(scale * xSpeed, scale * ySpeed, rot,
                                    new Rotation2d(currentAngleRadians))
                            : new ChassisSpeeds(xSpeed * scale, ySpeed * scale, rot),
                    centerOfRotation);
            return swerveModuleStates;
        } else {
            double scale = 1 / (1 + k);
            var swerveModuleStates = SwerveDriveDimensions.kinematics.toSwerveModuleStates(
                    fieldRelative
                            ? ChassisSpeeds.fromFieldRelativeSpeeds(scale * xSpeed, scale * ySpeed, scale * rot,
                                    new Rotation2d(currentAngleRadians))
                            : new ChassisSpeeds(xSpeed * scale, ySpeed * scale, rot * scale),
                    centerOfRotation);
            return swerveModuleStates;
        }

    }

    public static double computeMaxNorm(Translation2d[] translations, Translation2d centerOfRotation) {
        return Arrays.stream(translations)
                .map((translation) -> translation.minus(centerOfRotation))
                .mapToDouble(Translation2d::getNorm)
                .max()
                .orElseThrow(() -> new NoSuchElementException("No max norm."));
    }

    public static double angleDistance(double angle1, double angle2) {
        double rawDifference = angle2 - angle1;
        // Normalize the difference within the range [-pi, pi]
        double normalizedDifference = ((rawDifference + Math.PI) % (2 * Math.PI)) - Math.PI;

        // Adjust for cases where the difference is exactly -pi
        if (normalizedDifference < -Math.PI) {
            normalizedDifference += 2 * Math.PI;
        }
        return normalizedDifference;
    }
}