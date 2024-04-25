package frc.lib.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public interface DriveWeight {
    public default ChassisSpeeds getSpeeds(){return new ChassisSpeeds();};
    public default Translation2d getCenterOfRot(){return new Translation2d();};
    public default double getWeight(){return 1;};
    public default void setWeight(double weight){};
    public default double angle(){return 0;};
    public default boolean cancelCondition(){return false;};
    public default Command getAsCommand(){return new Command() {};};
}
