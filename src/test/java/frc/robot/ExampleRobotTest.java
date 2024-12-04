package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ExampleRobotTest {
    String[] words;

    @BeforeEach
    void setup() {
        words = new String[] { "Hello", "world" };
    }

    @Test
    void testExampleHello() {
        assertEquals("Hello", words[0]);
    }

    @Test
    void testExampleWorld() {
        assertEquals("world", words[1]);
    }
}
