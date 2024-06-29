package frc.lib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.lib.drive.SwerveAlgorithms;

class SwerveAlgorithmsTest {
    @Test
   void testMaxNorm(){
    assertEquals(0.40860165350864647, SwerveAlgorithms.maxNorm);
   }
 
}
