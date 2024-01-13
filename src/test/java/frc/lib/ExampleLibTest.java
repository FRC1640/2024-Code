package frc.lib;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ExampleLibTest {
    int[] numbers;

    @BeforeEach
    void setup() {
        numbers = new int[] { 1, 2 };
    }

    @Test
    void testExample1() {
        assertEquals(1, numbers[0]);
    }

    @Test
    void testExample2() {
        assertEquals(2, numbers[1]);
    }
}
