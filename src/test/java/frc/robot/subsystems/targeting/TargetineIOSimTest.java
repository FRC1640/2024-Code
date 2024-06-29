package frc.robot.subsystems.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class TargetineIOSimTest {
       private static Short motorEncoderValue;

    @Test
       void TargetineIOSimTest(){
        assertEquals(motorEncoderValue, TargetineIOSimTest.motorEncoderValue);
       }

}
