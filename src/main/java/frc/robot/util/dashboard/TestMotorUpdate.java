package frc.robot.util.dashboard;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TestMotorUpdate {
    
    private static GenericEntry motorSpeed;
    private static CANSparkMax realMotor;
    private static DCMotorSim simMotor;
    private static String type;

    public static void setEntry(GenericEntry speed) {
        motorSpeed = speed;
    }

    public static void setMotor(CANSparkMax motor) {
        type = "REAL";
        realMotor = motor;
        simMotor = null;
    }

    public static void setMotor(DCMotorSim motor) {
        type = "SIM";
        simMotor = motor;
        realMotor = null;
    }

    public void periodic() {
        switch (type) {
            case "REAL":
                realMotor.set(motorSpeed.getDouble(0));
            break;

            case "SIM":
                simMotor.setInputVoltage(motorSpeed.getDouble(0) * 12);
            break;
        }
    }


}
