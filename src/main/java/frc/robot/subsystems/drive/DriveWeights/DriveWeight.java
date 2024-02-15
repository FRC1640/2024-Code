package frc.robot.subsystems.drive.DriveWeights;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface DriveWeight {
    public default ChassisSpeeds getSpeeds(){return new ChassisSpeeds();};
    public default Translation2d getCenterOfRot(){return new Translation2d();};
    public default double getWeight(){return 1;};
    public default void setWeight(double weight){};
}
