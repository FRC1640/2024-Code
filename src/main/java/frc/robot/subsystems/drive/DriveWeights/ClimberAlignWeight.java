package frc.robot.subsystems.drive.DriveWeights;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DigitalInput;

public class ClimberAlignWeight implements DriveWeight {
    DigitalInput sensor1;
    DigitalInput sensor2;
    public ClimberAlignWeight(DigitalInput sensor1, DigitalInput sensor2){
        this.sensor1 = sensor1;
        this.sensor2 = sensor2;
    }

    @Override
    public ChassisSpeeds getSpeeds(){
        if (!sensor1.get() && !sensor1.get()){
            return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(-0.01, 0,0), new Rotation2d(0));
        } else if (!sensor1.get() && sensor1.get()){
            return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(0, 0, 0.05), new Rotation2d(0));
        } else if (sensor1.get() && !sensor1.get()){
            return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(0, 0, -0.05), new Rotation2d(0));
        } else {
            return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(0, 0,0), new Rotation2d(0));
        }
    }
    @Override
    public boolean cancelCondition() {
        return (sensor1.get() && sensor1.get());
    }
}
