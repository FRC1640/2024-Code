package frc.robot.subsystems.drive.DriveWeights;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import frc.lib.drive.DriveSubsystem;
import frc.lib.drive.DriveWeight;
import frc.lib.drive.SwerveAlgorithms;
import frc.robot.Constants.PIDConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.sensors.Gyro.Gyro;
import frc.robot.sensors.Vision.MLVision.MLVision;
import frc.robot.sensors.Vision.MLVision.Note;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class MLVisionWeight implements DriveWeight {


    private DriveSubsystem driveSubsystem;
    private Gyro gyro;
    PIDController pidr = PIDConstants.constructPID(PIDConstants.rotPID, "rotlock1");
    PIDController pidMoving = PIDConstants.constructPID(PIDConstants.rotMovingPID, "rotlockmoving1");
    private MLVision mlVision;
    // private Translation2d pose;
    PIDController pid = PIDConstants.constructPID(PIDConstants.driveForwardPID, "autodriveforward1");
    // TrapezoidProfile profile;
    // Constraints trapezoidConstraints;
    private IntakeSubsystem intakeSubsystem;
    Note note = null;
    boolean useLast;
    boolean end;
    long otherInitTime;
    Note lastNote;
    public MLVisionWeight(DriveSubsystem driveSubsystem, MLVision mlVision, Gyro gyro, IntakeSubsystem intakeSubsystem){
        this.mlVision = mlVision;
        this.driveSubsystem = driveSubsystem;
        // this.endState = endState;
        this.gyro = gyro;
        // this.pose = pose;
        // trapezoidConstraints = new Constraints(0.5, 0.5);
        this.intakeSubsystem = intakeSubsystem;
    }
    @Override
    public ChassisSpeeds getSpeeds() {
        note = mlVision.getClosestNote();

        if (note.confidence < 0.5 && !useLast){
            otherInitTime = System.currentTimeMillis();
            useLast = true;
        }
        if (note.confidence > 0.5 && driveSubsystem.getPose().getTranslation().getDistance(note.pose) < 1.5){
            useLast = false;
            otherInitTime = 0;
            lastNote = null;
        }
        
        if ((note.confidence < 0.5 || driveSubsystem.getPose().getTranslation().getDistance(note.pose) > 1.5
            ) && System.currentTimeMillis() - otherInitTime < 500 && lastNote != null) {
                note = lastNote;
                Logger.recordOutput("UseLastNote", true);
            }
        else{
            otherInitTime = 0;
            note = mlVision.getClosestNote();
            Logger.recordOutput("UseLastNote", false);
            lastNote = note;
            useLast = false;
        }
        Logger.recordOutput("NotePosCommand", note.pose);
        double angle = Math.atan2(note.pose.getY() - driveSubsystem.getPose().getY(),
            note.pose.getX() - driveSubsystem.getPose().getX());
        double o;
        
        o = pidr.calculate(-SwerveAlgorithms.angleDistance(driveSubsystem.getPose().getRotation().getRadians(),
                    (angle + Math.PI)), 0);

        if (Math.abs(o) < 0.005) {
            o = 0;
        }
        
        // double k;
        // double linearRotSpeed = Math.abs(o * SwerveAlgorithms.maxNorm);
        o = MathUtil.clamp(o, -1, 1);
        // setAngle = angle + gyro.getOffset();
        // o = MathUtil.clamp(o, -1, 1);
        ChassisSpeeds rToAngle = new ChassisSpeeds(0, 0, o);
        

        // double angle1 = new Rotation2d(pose.getX() - driveSubsystem.getPose().getX(), driveSubsystem.getPose().getY() - driveSubsystem.getPose().getY()).getRadians();
        double dist = driveSubsystem.getPose().getTranslation().getDistance(note.pose);
        double s = pid.calculate(dist, 0);
        s = Math.abs(s);
        s = MathUtil.clamp(s, -1, 1);
        if (Math.abs(s) < 0.01) {
            s = 0;
        }

        double xSpeed = (Math.cos(angle) * s) * 0.95;
        double ySpeed = (Math.sin(angle) * s) * 0.95;
        double offset = gyro.getOffset() - gyro.getRawAngleRadians() + driveSubsystem.getPose().getRotation().getRadians();
        ChassisSpeeds cspeeds = new ChassisSpeeds(xSpeed*Math.cos(offset)+ySpeed*Math.sin(offset), ySpeed*Math.cos(offset)-xSpeed*Math.sin(offset),0);
        ChassisSpeeds finalSpeed = rToAngle.plus(cspeeds);
    
        
        if (intakeSubsystem.hasNote() || note.pose == new Translation2d() || note.confidence < 0.5){
            return new ChassisSpeeds();
        }
        
        return finalSpeed;
    }
}