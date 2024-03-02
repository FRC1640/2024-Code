package frc.lib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrowsExactly;

import java.util.NoSuchElementException;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

import edu.wpi.first.math.geometry.Translation2d;

class SwerveAlgorithmsTest {
    @ParameterizedTest
    @CsvSource({
        "0, 0, 0", // 0 deg, 0 deg, expect 0 deg
        "0, 3.141592653589793, -3.141592653589793", // 0 deg, 180 deg, expect -180 deg
        "-3.141592653589793, 3.141592653589793, 0", // -180 deg, 180 deg, expect 0 deg
        "0.785398163, -0.785398163, -1.570796326", // 45 deg, -45 deg, expect -90 deg
        "-3.141592653589793, 1.5707963267948965, -1.5707963267948965", // -180 deg, 90 deg, expect -90 deg
        "3.141592653589793, -3.92699081698724125, -0.7853981633974483", // 180 deg, -270 deg, expect -90 deg
    })
    void testAngleDistance(double angle1, double angle2, double expected) {
        assertEquals(expected, SwerveAlgorithms.angleDistance(angle1, angle2));
    }

    @Test
    void testComputeMaxNorm() {
        Translation2d centerOfRotation = new Translation2d(0, 0);
        Translation2d[] translations = {new Translation2d(0, 1), new Translation2d(1, 0)};

        assertEquals(1.0, SwerveAlgorithms.computeMaxNorm(translations, centerOfRotation));

        translations[0] = new Translation2d(1, 1);
        assertEquals(Math.sqrt(2), SwerveAlgorithms.computeMaxNorm(translations, centerOfRotation));

        Translation2d[] exceptionTranslations = {};
        assertThrowsExactly(NoSuchElementException.class, () -> SwerveAlgorithms.computeMaxNorm(exceptionTranslations, centerOfRotation));
    }

    @Test
    void testMaxNorm() {
        assertEquals(0.40860165350864647, SwerveAlgorithms.maxNorm);
    }
}
