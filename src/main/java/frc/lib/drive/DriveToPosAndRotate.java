package frc.lib.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PIDConstants;
import frc.robot.sensors.Gyro.Gyro;

public class DriveToPosAndRotate extends Command{

    private DriveSubsystem driveSubsystem;
    private Gyro gyro;
    private Supplier<Translation2d> endState;
    PIDController pidr = PIDConstants.constructPID(PIDConstants.rotPID, "rotlock");
    PIDController pidMoving = PIDConstants.constructPID(PIDConstants.rotMovingPID, "rotlockmoving");

    public DriveToPosAndRotate(DriveSubsystem driveSubsystem, Supplier<Translation2d> endState, Gyro gyro){
        this.driveSubsystem = driveSubsystem;
        this.endState = endState;
        this.gyro = gyro;

        addRequirements(driveSubsystem);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.driveDoubleCone(0, 0, 0, false, new Translation2d());
    }

    @Override
    public void execute() {
        double angle = Math.atan2(endState.get().getY() - driveSubsystem.getPose().getY(),
            endState.get().getX() - driveSubsystem.getPose().getX()) - gyro.getOffset();
        double o;
        
        o = pidr.calculate(-SwerveAlgorithms.angleDistance(driveSubsystem.getPose().getRotation().getRadians(),
                    (angle + gyro.getOffset())), 0);

        if (Math.abs(o) < 0.005) {
            o = 0;
        }
        
        double k;
        double linearRotSpeed = Math.abs(o * SwerveAlgorithms.maxNorm);
        o = MathUtil.clamp(o, -1, 1);
        // setAngle = angle + gyro.getOffset();
        // o = MathUtil.clamp(o, -1, 1);
        ChassisSpeeds rToAngle = new ChassisSpeeds(0, 0, -o);
        driveSubsystem.driveDoubleCone(rToAngle.vxMetersPerSecond, rToAngle.vyMetersPerSecond, rToAngle.omegaRadiansPerSecond / 6, true, new Translation2d());
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
