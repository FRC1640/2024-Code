package frc.lib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrowsExactly;

import java.util.NoSuchElementException;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

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
        assertEquals(0.4086, SwerveAlgorithms.maxNorm, 0.0001);
    }

    @Test
    void testRawSpeeds() {
        Rotation2d angle = new Rotation2d();

        SwerveModuleState[] states = SwerveAlgorithms.rawSpeeds(0, 0, 0);
        assertEquals(4, states.length);
        for (int i = 0; i < states.length; i++) {
            assertSwerveModuleState(states[i], 0, angle.getDegrees());
        }

        states = SwerveAlgorithms.rawSpeeds(2, 0, 0);
        assertEquals(4, states.length);
        for (int i = 0; i < states.length; i++) {
            assertSwerveModuleState(states[i], 2, angle.getDegrees());
        }

        states = SwerveAlgorithms.rawSpeeds(2, 2, 0);
        angle = Rotation2d.fromDegrees(45);
        assertEquals(4, states.length);
        for (int i = 0; i < states.length; i++) {
            assertSwerveModuleState(states[i], 2 * Math.sqrt(2), angle.getDegrees());
        }

        states = SwerveAlgorithms.rawSpeeds(2, 2, Math.PI / 6);
        assertEquals(4, states.length);
        assertSwerveModuleState(states[0], 2.84, 49.33);
        assertSwerveModuleState(states[1], 3.04, 45);
        assertSwerveModuleState(states[2], 2.61, 45);
        assertSwerveModuleState(states[3], 2.84, 40.67);
    }

    private void assertSwerveModuleState(SwerveModuleState state, double expectedSpeed, double expectedAngle) {
        assertNotNull(state);
        assertEquals(expectedSpeed, state.speedMetersPerSecond, 0.01);
        assertNotNull(state.angle);
        assertEquals(expectedAngle, state.angle.getDegrees(), 0.01);
    }
}
