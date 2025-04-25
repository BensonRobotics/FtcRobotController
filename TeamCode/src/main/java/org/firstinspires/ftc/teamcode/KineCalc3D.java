package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class KineCalc3D {
    /**
     * JOINT 0 IS SWIVEL BASE
     * JOINT 1 IS SHOULDER JOINT
     * JOINT 2 IS ELBOW JOINT
     * JOINT 3 IS WRIST JOINT
     **/

    private float[] L;
    private double l;
    private double[] Jmin;
    private double[] Jmax;
    private ElapsedTime controllerTimer;

    public void initialize(float[] lengths, double[] minAngles, double[] maxAngles) {
        this.L = lengths;
        this.l = lengths[0] + lengths[1];
        this.Jmin = minAngles;
        this.Jmax = maxAngles;
        this.controllerTimer = new ElapsedTime();
    }

    public double[] jointsFromVector(VectorF vector) {
        // Calculate joint angles from vector and gamma (pitch) using inverse kinematics
        double R = Math.atan2(vector.get(1), vector.get(0));
        double M = Math.hypot(vector.get(0), vector.get(1));

        double h = Math.cos(vector.get(3)) * L[2];
        double k = Math.sin(vector.get(3)) * L[2];

        double range = Math.min(1, 0.99 * l /
                Math.hypot(M - h, vector.get(2) - k));
        double Ml = h + (M - h) * range;
        double zl = k + (vector.get(2) - k) * range;

        double x3 = Ml - L[2] * Math.cos(R);
        double y3 = zl - L[2] * Math.sin(R);
        double C = Math.hypot(x3, y3);

        double a = Math.acos((Math.pow(L[0], 2) + Math.pow(L[1], 2) - Math.pow(C, 2)) /
                (2 * L[0] * L[1]));
        double B = Math.acos((Math.pow(L[0], 2) + Math.pow(C, 2) - Math.pow(L[1], 2)) /
                (2 * L[0] * C));

        double[] J = new double[4];
        J[0] = R;
        J[1] = Math.atan2(y3, x3) + B;
        J[2] = -1 * (180 - a);
        J[3] = vector.get(3) - J[1] - J[2];

        for (int i = 0; i < J.length; i++) {
            J[i] = Math.max(Math.min(J[i], Math.toRadians(Jmax[i])), Math.toRadians(Jmin[i]));
        }
        return J;
    }

    public VectorF vectorFromJointAngles(double[] angles) {
        // Calculate vector from joint angles
        // Gamma never gets constrained, so don't worry about it

        // This section is planar with the robot arm's theta angle
        double x2 = L[0] * Math.cos(angles[1]);
        double x3 = x2 + L[1] * Math.cos(angles[1] + angles[2]);
        double xe = x3 + L[2] * Math.cos(angles[1] + angles[2] + angles[3]);
        double y2 = L[0] * Math.sin(angles[1]);
        double y3 = y2 + L[1] * Math.sin(angles[1] + angles[2]);
        double ye = y3 + L[2] * Math.sin(angles[1] + angles[2] + angles[3]);

        // This section is the planar part rotated into 3D space
        float x = (float) (xe * Math.cos(angles[0]));
        float y = (float) (xe * Math.sin(angles[0]));
        float z = (float) ye;
        float g = (float) (angles[1] + angles[2] + angles[3]);

        return new VectorF(x, y, z, g);
    }

    public void moveVectorWithJoystick(VectorF vector, VectorF joystick,
                                       double moveSpeed, double gammaSpeed)
    {
        int msInterval = 10;
        if (controllerTimer.milliseconds() > msInterval) {
            double moveMult = moveSpeed / (1000.0 / msInterval);
            double gammaMult = gammaSpeed / (60000.0 / msInterval);
            for (int i = 0; i < 4; i++) {
                vector.put(i, (float) (vector.get(i) + joystick.get(i) *
                        (i == 3 ? gammaMult : moveMult)));
            }
            controllerTimer.reset();
        }
    }

}
