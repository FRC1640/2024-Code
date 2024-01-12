package frc.lib.swerve;

import java.util.Arrays;
import java.util.NoSuchElementException;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveDriveDimensions;

public class SwerveAlgorithms {
    public static SwerveModuleState[] desaturated(double xSpeed, double ySpeed, double rot, Supplier<Double> currentAngleRadians, boolean fieldRelative){
        var swerveModuleStates = SwerveDriveDimensions.kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
                new Rotation2d(currentAngleRadians.get()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveDriveDimensions.maxSpeed);
        return swerveModuleStates;
    }
    public static SwerveModuleState[] doubleCone(double xSpeed, double ySpeed, double rot, Supplier<Double> currentAngleRadians, boolean fieldRelative){
        double translationalSpeed = Math.hypot(xSpeed, ySpeed);
        double linearRotSpeed = Math.abs(rot * computeMaxNorm(SwerveDriveDimensions.positions, new Translation2d(0,0)));
        double k;
        if (linearRotSpeed == 0 || translationalSpeed == 0){
            k = 0;
        }
        else{
            k = Math.min(translationalSpeed / linearRotSpeed, linearRotSpeed / translationalSpeed);
        }
        double scale = 1 / (1 + k);
        var swerveModuleStates = SwerveDriveDimensions.kinematics.toSwerveModuleStates(
            fieldRelative 
                ? ChassisSpeeds.fromFieldRelativeSpeeds(scale * xSpeed, scale * ySpeed, scale * rot, 
                new Rotation2d(currentAngleRadians.get())) 
                : new ChassisSpeeds(xSpeed * scale, ySpeed * scale, rot * scale));
        return swerveModuleStates;
    }

    
    public static double computeMaxNorm(Translation2d[] translations, Translation2d centerOfRotation) {
        return Arrays.stream(translations)
            .map((translation) -> translation.minus(centerOfRotation))
            .mapToDouble(Translation2d::getNorm)
            .max()
            .orElseThrow(() -> new NoSuchElementException("No max norm."));
    }
}