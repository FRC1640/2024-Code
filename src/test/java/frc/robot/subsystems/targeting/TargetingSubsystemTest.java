package frc.robot.subsystems.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class TargetingSubsystemTest {
    TargetingSubsystem ts= new TargetingSubsystem(new TargetingIOSparkMax(), ()->0.0);
    @Test
    void testGetAngleVoltage (){
        assertEquals(ts.inputs.targetingVoltage, ts.getAngleVoltage());
    }
}
