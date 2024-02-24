package frc.robot.util.dashboard;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.sensors.Resolver;

public class TestMotorUpdate {
    
    private GenericEntry motorSpeed;

    private MotorType type;
    private CANSparkMax realMotor;
    private DCMotorSim simMotor;

    private MotorGroupType groupType;
    private CANSparkMax[] realMotors;
    private DCMotorSim[] simMotors;

    private MotorLimitStyle limitStyle;
    private double lowerLimit;
    private double upperLimit;
    
    private MotorEncoderType encoderType;
    private Resolver resolver;
    private Resolver[] resolvers;
    private double simulatedPosition;

    /**
     * Whether the motor is simulated or real.
     */
    private enum MotorType {
        REAL, SIM
    }

    /**
     * Whether this {@code TestMotorUpdate} controls a single motor or a group.
     */
    private enum MotorGroupType {
        SINGLE, GROUP
    }

    /**
     * Whether this object's motor uses limits or is free-moving.
     */
    private enum MotorLimitStyle {
        FREE, LIMITED
    }

    /**
     * Whether this object's motor uses a built-in encoder or a {@code Resolver}.
     */
    public enum MotorEncoderType {
        INBUILT, RESOLVER, SIMULATED
    }

    /**
     * Periodically updates a motor to the percent output broadcasted from Shuffleboard.
     * 
     * @param realMotor {@code CANSparkMax} to update.
     * @param entry {@code GenericEntry} in which to find the percent output.
     */
    public TestMotorUpdate(CANSparkMax realMotor, GenericEntry entry) {
        this.realMotor = realMotor;
        motorSpeed = entry;
        this.type = MotorType.REAL;
        this.groupType = MotorGroupType.SINGLE;
        this.limitStyle = MotorLimitStyle.FREE;
    }

    /**
     * Periodically updates a motor to the percent output broadcasted from Shuffleboard.
     * 
     * @param simMotor {@code DCMotorSim} to update.
     * @param entry {@code GenericEntry} in which to find the percent output.
     */
    public TestMotorUpdate(DCMotorSim simMotor, GenericEntry entry) {
        this.simMotor = simMotor;
        motorSpeed = entry;
        this.type = MotorType.SIM;
        this.groupType = MotorGroupType.SINGLE;
        this.limitStyle = MotorLimitStyle.FREE;
    }

    /**
     * Periodically updates a group of motors to the percent output broadcasted from Shuffleboard.
     * 
     * @param realMotors {@code CANSparkMax[]} of motors to update.
     * @param entry {@code GenericEntry} in which to find the percent output.
     */
    public TestMotorUpdate(CANSparkMax[] realMotors, GenericEntry entry) {
        this.realMotors = realMotors;
        motorSpeed = entry;
        this.type = MotorType.REAL;
        this.groupType = MotorGroupType.GROUP;
        this.limitStyle = MotorLimitStyle.FREE;
    }

    /**
     * Periodically updates a group of motors to the percent output broadcasted from Shuffleboard.
     * 
     * @param simMotors {@code DCMotorSim[]} of motors to update.
     * @param entry {@code GenericEntry} in which to find the percent output.
     */
    public TestMotorUpdate(DCMotorSim[] simMotors, GenericEntry entry) {
        this.simMotors = simMotors;
        motorSpeed = entry;
        this.type = MotorType.SIM;
        this.groupType = MotorGroupType.GROUP;
        this.limitStyle = MotorLimitStyle.FREE;
    }

    /**
     * Periodically updates a motor to the percent output broadcasted from Shuffleboard, not moving out of the motor's limits.
     * 
     * @param realMotor {@code CANSparkMax} to update.
     * @param entry {@code GenericEntry} in which to find the percent output.
     * @param lowerLimit Lower limit of the motor.
     * @param upperLimit Upper limit of the motor.
     */
    public TestMotorUpdate(CANSparkMax realMotor, GenericEntry entry, double lowerLimit, double upperLimit) {
        this.realMotor = realMotor;
        motorSpeed = entry;
        this.type = MotorType.REAL;
        this.groupType = MotorGroupType.SINGLE;
        this.lowerLimit = lowerLimit;
        this.upperLimit = upperLimit;
        this.limitStyle = MotorLimitStyle.LIMITED;
        this.encoderType = MotorEncoderType.INBUILT;
    }

    /**
     * Periodically updates a motor to the percent output broadcasted from Shuffleboard, not moving out of the motor's limits.
     * 
     * @param realMotor {@code CANSparkMax} to update.
     * @param entry {@code GenericEntry} in which to find the percent output.
     * @param lowerLimit Lower limit of the motor.
     * @param upperLimit Upper limit of the motor.
     * @param resolver {@code Resolver} from the motor.
     */
    public TestMotorUpdate(CANSparkMax realMotor, GenericEntry entry, double lowerLimit, double upperLimit, Resolver resolver) {
        this.realMotor = realMotor;
        motorSpeed = entry;
        this.type = MotorType.REAL;
        this.groupType = MotorGroupType.SINGLE;
        this.lowerLimit = lowerLimit;
        this.upperLimit = upperLimit;
        this.limitStyle = MotorLimitStyle.LIMITED;
        this.encoderType = MotorEncoderType.RESOLVER;
        this.resolver = resolver;
    }

    /**
     * Periodically updates a motor to the percent output broadcasted from Shuffleboard, not moving out of the motor's limits.
     * 
     * @param simMotor {@code DCMotorSim} to update.
     * @param entry {@code GenericEntry} in which to find the percent output.
     * @param lowerLimit Lower limit of the motor.
     * @param upperLimit Upper limit of the motor.
     * @param position Position of the motor.
     */
    public TestMotorUpdate(DCMotorSim simMotor, GenericEntry entry, double lowerLimit, double upperLimit, double position) {
        this.simMotor = simMotor;
        motorSpeed = entry;
        this.type = MotorType.SIM;
        this.groupType = MotorGroupType.SINGLE;
        this.lowerLimit = lowerLimit;
        this.upperLimit = upperLimit;
        this.limitStyle = MotorLimitStyle.LIMITED;
        this.encoderType = MotorEncoderType.SIMULATED;
        this.simulatedPosition = position;
    }

    /**
     * Periodically updates a group of motors to the percent output broadcasted from Shuffleboard, staying within limits.
     * 
     * @param realMotors {@code CANSparkMax[]} of motors to update.
     * @param entry {@code GenericEntry} in which to find the percent output.
     * @param lowerLimit Lower limit of the motor.
     * @param upperLimit Upper limit of the motor.
     */
    public TestMotorUpdate(CANSparkMax[] realMotors, GenericEntry entry, double lowerLimit, double upperLimit) {
        this.realMotors = realMotors;
        motorSpeed = entry;
        this.type = MotorType.REAL;
        this.groupType = MotorGroupType.GROUP;
        this.lowerLimit = lowerLimit;
        this.upperLimit = upperLimit;
        this.limitStyle = MotorLimitStyle.LIMITED;
        this.encoderType = MotorEncoderType.INBUILT;
    }

    /**
     * Periodically updates a group of motors to the percent output broadcasted from Shuffleboard, staying within limits.
     * 
     * @param realMotors {@code CANSparkMax[]} of motors to update.
     * @param entry {@code GenericEntry} in which to find the percent output.
     * @param lowerLimit Lower limit of the motor.
     * @param upperLimit Upper limit of the motor.
     * @param resolvers {@code Resolver[]} of motor encoders. <strong> MUST </strong> be of same length as realMotors.
     */
    public TestMotorUpdate(CANSparkMax[] realMotors, GenericEntry entry, double lowerLimit, double upperLimit, Resolver[] resolvers) {
        this.realMotors = realMotors;
        motorSpeed = entry;
        this.type = MotorType.REAL;
        this.groupType = MotorGroupType.GROUP;
        this.lowerLimit = lowerLimit;
        this.upperLimit = upperLimit;
        this.limitStyle = MotorLimitStyle.LIMITED;
        this.encoderType = MotorEncoderType.RESOLVER;
    }

    /**
     * Periodically updates a group of motors to the percent output broadcasted from Shuffleboard, staying within limits.
     * 
     * @param realMotors {@code DCMotorSim[]} of motors to update.
     * @param entry {@code GenericEntry} in which to find the percent output.
     * @param lowerLimit Lower limit of the motors.
     * @param upperLimit Upper limit of the motors.
     * @param position Position of the motors.
     */
    public TestMotorUpdate(DCMotorSim[] simMotors, GenericEntry entry, double lowerLimit, double upperLimit, double position) {
        this.simMotors = simMotors;
        motorSpeed = entry;
        this.type = MotorType.SIM;
        this.groupType = MotorGroupType.GROUP;
        this.lowerLimit = lowerLimit;
        this.upperLimit = upperLimit;
        this.limitStyle = MotorLimitStyle.LIMITED;
        this.encoderType = MotorEncoderType.SIMULATED;
        this.simulatedPosition = position;
    }

    /**
     * Periodically updates any motor passed into the {@code TestMotorUpdate}'s constructor, accounting for
     * all combinations of motor type, group control, limits, and encoder types.
     */
    public void periodic() {
        switch (type) {
            case REAL:
                switch (groupType) {
                    case SINGLE:
                        switch (limitStyle) {
                            case FREE:
                               realMotor.set(motorSpeed.getDouble(0)); 
                            break;

                            case LIMITED:
                                switch (encoderType) {
                                    case INBUILT:
                                        if (realMotor.getEncoder().getPosition() < lowerLimit) {
                                            realMotor.set(Math.max(motorSpeed.getDouble(0), 0));
                                        }
                                        if (realMotor.getEncoder().getPosition() > upperLimit) {
                                            realMotor.set(Math.min(motorSpeed.getDouble(0), 0));
                                        }
                                        else {
                                            realMotor.set(motorSpeed.getDouble(0));
                                        }
                                    break;
                                    
                                    case RESOLVER:
                                        if (resolver.getD() < lowerLimit) {
                                            realMotor.set(Math.max(motorSpeed.getDouble(0), 0));
                                        }
                                        if (resolver.getD() > upperLimit) {
                                            realMotor.set(Math.min(motorSpeed.getDouble(0), 0));
                                        }
                                        else {
                                            realMotor.set(motorSpeed.getDouble(0));
                                        }
                                    break;

                                    default:

                                    break;
                                }
                            break;
                        }
                    break;

                    case GROUP:
                        switch (limitStyle) {
                            case FREE:
                                for (int i = 0; i < realMotors.length; i++) {
                                    realMotors[i].set(motorSpeed.getDouble(0));
                                }
                            break;

                            case LIMITED:
                                switch (encoderType) {
                                    case INBUILT:
                                        for (int i = 0; i < realMotors.length; i++) {
                                            if (realMotors[i].getEncoder().getPosition() < lowerLimit) {
                                                realMotors[i].set(Math.max(motorSpeed.getDouble(0), 0));
                                            }
                                            if (realMotors[i].getEncoder().getPosition() > upperLimit) {
                                                realMotors[i].set(Math.min(motorSpeed.getDouble(0), 0));
                                            }
                                            else {
                                                realMotors[i].set(motorSpeed.getDouble(0));
                                            }
                                        }
                                    break;

                                    case RESOLVER:
                                        for (int i = 0; i < realMotors.length; i++) {
                                            if (resolvers[i].getD() < lowerLimit) {
                                                realMotors[i].set(Math.max(motorSpeed.getDouble(0), 0));
                                            }
                                            if (resolvers[i].getD() > upperLimit) {
                                                realMotors[i].set(Math.min(motorSpeed.getDouble(0), 0));
                                            }
                                            else {
                                                realMotors[i].set(motorSpeed.getDouble(0));
                                            }
                                        }
                                    break;

                                    default:

                                    break;
                                }
                            break;
                        }
                    break;
                    }
            break;

            case SIM:
                switch (groupType) {
                    case SINGLE:
                        switch (limitStyle) {
                            case FREE:
                                simMotor.setInputVoltage(motorSpeed.getDouble(0) * 12);
                            break;

                            case LIMITED:
                                if (simulatedPosition < lowerLimit) {
                                    simMotor.setInputVoltage(Math.max(motorSpeed.getDouble(0), 0) * 12);
                                }
                                if (simulatedPosition > upperLimit) {
                                    simMotor.setInputVoltage(Math.min(motorSpeed.getDouble(0), 0) * 12);
                                }
                                else {
                                    simMotor.setInputVoltage(motorSpeed.getDouble(0) * 12);
                                }
                            break;
                        }
                    break;

                    case GROUP:
                        switch (limitStyle) {
                            case FREE:
                                for (int i = 0; i < simMotors.length; i++) {
                                    simMotors[i].setInputVoltage(motorSpeed.getDouble(0) * 12);
                                }
                            break;

                            case LIMITED:
                                for (int i = 0; i < simMotors.length; i++) {
                                    if (simulatedPosition < lowerLimit) {
                                        simMotors[i].setInputVoltage(Math.max(motorSpeed.getDouble(0), 0) * 12);
                                    }
                                    if (simulatedPosition > upperLimit) {
                                        simMotors[i].setInputVoltage(Math.min(motorSpeed.getDouble(0), 0) * 12);
                                    }
                                    else {
                                        simMotors[i].setInputVoltage(motorSpeed.getDouble(0) * 12);
                                    }
                                }
                            break;
                        }
                    break;
                }
            break;
        }
    }
}
