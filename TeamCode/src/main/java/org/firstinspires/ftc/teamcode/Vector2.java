package org.firstinspires.ftc.teamcode;

public class Vector2 {
    public float x;
    public float y;

    public Vector2(float x1, float y1) {
        x = x1;
        y = y1;
    }

    // rotate a vector about it's origin using an angle (double) in radians (copy pasted from stack overflow)
    public Vector2 rotate(double angle) {

        float x1 = (float) (x * Math.cos(angle) - y * Math.sin(angle));

        float y1 = (float) (x * Math.sin(angle) + y * Math.cos(angle));

        return new Vector2(x1, y1);
    }

    // return the magnitude of a Vector2 using the pythagorean theorem
    public float magnitude() {
        return (float) Math.sqrt((x * x) + ((y * y)));
    }
}
