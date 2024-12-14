package frc.lib.drive;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.SwerveDriveDimensions;
import frc.robot.sensors.Vision.MLVision.MLVision;
import frc.robot.sensors.Vision.MLVision.Note;

public class DriveToPosAndRotateCurve extends Command{

    private DriveSubsystem driveSubsystem;
    PIDController pidr = PIDConstants.constructPID(PIDConstants.rotPID, "rotlock1");
    PIDController pidMoving = PIDConstants.constructPID(PIDConstants.rotMovingPID, "rotlockmoving1");
    Note lastNote = null;
    private MLVision mlVision;
    // private Translation2d pose;
    PIDController pid = PIDConstants.constructPID(PIDConstants.driveForwardPID, "autodriveforward1");
    TrapezoidProfile profileX;
    Constraints trapezoidConstraintsX;
    TrapezoidProfile profileY;
    Constraints trapezoidConstraintsY;
    double time = 0;
    TrapezoidProfile.State xSpeed;
    TrapezoidProfile.State ySpeed;

    double lastTime = 0;
    public DriveToPosAndRotateCurve(DriveSubsystem driveSubsystem, MLVision mlVision){
        this.driveSubsystem = driveSubsystem;
        this.mlVision = mlVision;
        // this.pose = pose;
        trapezoidConstraintsX = new Constraints(0.25, 0.25);
        
        profileX = new TrapezoidProfile(trapezoidConstraintsX);

        trapezoidConstraintsY = new Constraints(0.25, 0.25);
        
        profileY= new TrapezoidProfile(trapezoidConstraintsX);

        addRequirements(driveSubsystem);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.driveDoubleCone(0, 0, 0, false, new Translation2d());
    }

    @Override
    public void execute() {
        Note note = null;
        
        time += System.currentTimeMillis() / 1000 - lastTime;
        if (lastNote != null){
            if (mlVision.getClosestNote().pose.getDistance(lastNote.pose) < 0.25){
                note = mlVision.getClosestNote();
                time = 0;
                xSpeed = new TrapezoidProfile.State(driveSubsystem.getPose().getX(), driveSubsystem.getChassisSpeeds().vxMetersPerSecond);
                ySpeed = new TrapezoidProfile.State(driveSubsystem.getPose().getY(), driveSubsystem.getChassisSpeeds().vyMetersPerSecond);
            }
            else{
                note = lastNote;
            }
        }
        else{
            note = mlVision.getClosestNote();
        }
        Logger.recordOutput("TrapezoidTime", time);
        note = new Note(new Translation2d(3, 3), 1);
        double angle = Math.atan2(driveSubsystem.getChassisSpeeds().vyMetersPerSecond, driveSubsystem.getChassisSpeeds().vxMetersPerSecond);
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
        xSpeed = profileX.calculate(time, new TrapezoidProfile.State(driveSubsystem.getPose().getX(), driveSubsystem.getChassisSpeeds().vxMetersPerSecond), new TrapezoidProfile.State(note.pose.getX(), 0));
        ySpeed = profileY.calculate(time, new TrapezoidProfile.State(driveSubsystem.getPose().getY(), driveSubsystem.getChassisSpeeds().vyMetersPerSecond), new TrapezoidProfile.State(note.pose.getY(), 0));
        ChassisSpeeds cspeeds = new ChassisSpeeds(-xSpeed.velocity,-ySpeed.velocity,0);
        ChassisSpeeds finalSpeed = cspeeds.plus(rToAngle);
        driveSubsystem.driveDoubleConePercent(finalSpeed.vxMetersPerSecond / SwerveDriveDimensions.maxSpeed, 
            finalSpeed.vyMetersPerSecond / SwerveDriveDimensions.maxSpeed, 
            finalSpeed.omegaRadiansPerSecond, true, new Translation2d(), false);
        lastNote = note;
        lastTime = System.currentTimeMillis();
    }

    @Override
    public void initialize() {
        time = 0;
        lastTime = System.currentTimeMillis() / 1000;
        xSpeed = new TrapezoidProfile.State(driveSubsystem.getPose().getX(), driveSubsystem.getChassisSpeeds().vxMetersPerSecond);
        ySpeed = new TrapezoidProfile.State(driveSubsystem.getPose().getY(), driveSubsystem.getChassisSpeeds().vyMetersPerSecond);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
