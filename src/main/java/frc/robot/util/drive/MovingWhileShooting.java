package frc.robot.util.drive;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.sensors.Gyro.Gyro;
import frc.robot.subsystems.targeting.TargetingSubsystem;

public class MovingWhileShooting {
    Supplier<Pose2d> goalPose;
    Supplier<Pose2d> getPose;
    Gyro gyro;
    Supplier<ChassisSpeeds> speeds;
    private TargetingSubsystem targetingSubsystem;
    public MovingWhileShooting(Gyro gyro, Supplier<Pose2d> goalPose, Supplier<Pose2d> getPose, 
            Supplier<ChassisSpeeds> speeds, TargetingSubsystem targetingSubsystem) {
        this.goalPose = goalPose;
        this.gyro = gyro;
        this.getPose = getPose;
        this.speeds = speeds;
        this.targetingSubsystem = targetingSubsystem;
    }

    public double getAngleToGoal() {
        double angle = Math.atan2(goalPose.get().getY() - getPose.get().getY(),
                goalPose.get().getX() - getPose.get().getX()) - gyro.getOffset();
        return angle + gyro.getOffset();
    }

    public double getDistance() {
        return new Translation3d(goalPose.get().minus(getPose.get()).getX(),
                goalPose.get().minus(getPose.get()).getY(), 2.11).getNorm();
    }

    public double getAngleFromDistance() {
        return targetingSubsystem.distToAngle(()->getDistance());
    }

    public double getSpeedFromDistance(){
        return 15;
    }

    public double speedToPercentOutput(){
        return getNewShotSpeed(); // TODO: conversion
    }
    

    public double getNewRobotAngle() {
        double V = getSpeedFromDistance();
        double phiv = Math.toRadians(getAngleFromDistance());
        double phih = getAngleToGoal();
        double vy = speeds.get().vyMetersPerSecond;
        double vx = speeds.get().vxMetersPerSecond;
        Logger.recordOutput("Drive/MovingWhileShooting/AngleDistance", 
            Math.toDegrees(Math.abs(Math.atan2(V * Math.cos(phiv) * Math.sin(phih) + vy, V * Math.cos(phiv) * Math.cos(phih) + vx) - phih)));
        return Math.atan2(V * Math.cos(phiv) * Math.sin(phih) + vy, V * Math.cos(phiv) * Math.cos(phih) + vx);
    }

    public double getNewTargetingAngle() {
        double V = getSpeedFromDistance();
        double phiv = Math.toRadians(getAngleFromDistance());
        double phih = getAngleToGoal();
        double vy = speeds.get().vyMetersPerSecond;
        double vx = speeds.get().vxMetersPerSecond;
        double thetah = getNewRobotAngle();
        if (vx == 0 && phih == Math.PI / 2){
            return phiv;
        }
        return Math.atan2(V * Math.sin(phiv) * Math.cos(thetah), V * Math.cos(phiv) * Math.cos(phih) + vx);
    }

    public double getNewShotSpeed() {
        double V = getSpeedFromDistance();
        double phiv = Math.toRadians(getAngleFromDistance());
        double phih = getAngleToGoal();
        double vy = speeds.get().vyMetersPerSecond;
        double vx = speeds.get().vxMetersPerSecond;
        double thetah = getNewRobotAngle();
        return V * Math.sin(phiv) / Math.sin(thetah);
    }
}
