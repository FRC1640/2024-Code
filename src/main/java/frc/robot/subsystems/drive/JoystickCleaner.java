package frc.robot.subsystems.drive;

import java.lang.Math;

public class JoystickCleaner {
    private double x;
    private double y;

    public JoystickCleaner() {
        this.x = 0;
        this.y = 0;
    }

    public void applyDeadband(double lowerDb, double upperDb) {
        double r = Math.sqrt(x*x + y*y);
        if (r <= lowerDb) {
            this.x = 0;
            this.y = 0;
        } else if (r >= 1 - upperDb) {
            x /= r;
            y /= r;
        } else {
            x *= (r - lowerDb) / (1 - upperDb - lowerDb) / r;
            y *= (r - lowerDb) / (1 - upperDb - lowerDb) / r;
        }
    }

    public void applySensitivity(double sensitivity) {
        if (sensitivity <= 0) {
            throw new IllegalArgumentException("Sensitivity must be positive.");
        }
        double r = Math.sqrt(x*x + y*y);
        x *= Math.pow(r, 1 / sensitivity - 1);
        y *= Math.pow(r, 1 / sensitivity - 1);
    }

    public void applyCirclify() {
        x *= Math.sqrt(1 - y*y / 2);
        y *= Math.sqrt(1 - y*y / 2);
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }
}