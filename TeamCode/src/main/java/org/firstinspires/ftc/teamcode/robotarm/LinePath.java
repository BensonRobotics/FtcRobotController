package org.firstinspires.ftc.teamcode.robotarm;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class LinePath implements Path {
    VectorF startPoint;
    VectorF endPoint;
    VectorF errorVector;
    public LinePath(VectorF startPoint, VectorF endPoint) {
        this.startPoint = startPoint;
        this.endPoint = endPoint;
        this.errorVector = endPoint.subtracted(startPoint);
    }

    public VectorF getPoint(double t) {
        t = Math.min(Math.max(t, 0), 1);
        return startPoint.added(errorVector.multiplied((float) t));
    }

    public double getDistance() {
        return errorVector.magnitude();
    }
}
