package frc.robot.util.dashboard;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TestMotorUpdate {
    
    private GenericEntry motorSpeed;
    private CANSparkMax realMotor;
    private CANSparkMax[] realMotors;
    private DCMotorSim simMotor;
    private DCMotorSim[] simMotors;
    private MotorType type;
    private MotorGroupType groupType;

    private enum MotorType {
        REAL, SIM
    }

    private enum MotorGroupType {
        SINGLE, GROUP
    }

    public TestMotorUpdate(CANSparkMax realMotor, GenericEntry entry) {
        this.realMotor = realMotor;
        motorSpeed = entry;
        this.type = MotorType.REAL;
        this.groupType = groupType.SINGLE;
    }

    public TestMotorUpdate(DCMotorSim simMotor, GenericEntry entry) {
        this.simMotor = simMotor;
        motorSpeed = entry;
        this.type = MotorType.SIM;
        this.groupType = groupType.SINGLE;
    }

    public TestMotorUpdate(CANSparkMax[] realMotors, GenericEntry entry) {
        this.realMotors = realMotors;
        motorSpeed = entry;
        this.type = MotorType.REAL;
        this.groupType = groupType.GROUP;
    }

    public TestMotorUpdate(DCMotorSim[] simMotors, GenericEntry entry) {
        this.simMotors = simMotors;
        motorSpeed = entry;
        this.type = MotorType.SIM;
        this.groupType = groupType.GROUP;
    }

    public void periodic() {
        switch (type) {
            case REAL:
                switch (groupType) {
                    case SINGLE:
                        realMotor.set(motorSpeed.getDouble(0));
                    break;

                    case GROUP:
                        for (int i = 0; i < realMotors.length; i++) {
                            realMotors[i].set(motorSpeed.getDouble(0));
                        }
                    break;
                }
            break;

            case SIM:
                switch (groupType) {
                    case SINGLE:
                        simMotor.setInputVoltage(motorSpeed.getDouble(0) * 12);
                    break;

                    case GROUP:
                        for (int i = 0; i < simMotors.length; i++) {
                            simMotors[i].setInputVoltage(motorSpeed.getDouble(0) * 12);
                        }
                    break;
                }
            break;
        }
    }


}
