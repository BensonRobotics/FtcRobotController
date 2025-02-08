package org.firstinspires.ftc.teamcode.opmodes;

public class Vector2 {
    public double x;
    public double y;

    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    // rotate a vector about it's origin using an angle (double) in radians (copy pasted from stack overflow)
    public Vector2 Rotate(double angle) {
        double x1 = (x * Math.cos(angle) - y * Math.sin(angle));

        double y1 = (x * Math.sin(angle) + y * Math.cos(angle));

        return new Vector2(x1, y1);
    }

    // return the magnitude of a Vector2 using the pythagorean theorem
    public double Magnitude() {
        return  Math.hypot(x, y);
    }

    public String Value() {
        return ("(" + x + " " + y + ")");
    }
}
