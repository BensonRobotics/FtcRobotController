package org.firstinspires.ftc.teamcode.robotarm;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import au.edu.federation.utils.Vec3f;

public class KinematicCalculator {
    /**
     * JOINT 0 IS SWIVEL BASE
     * JOINT 1 IS SHOULDER JOINT
     * JOINT 2 IS ELBOW JOINT
     * JOINT 3 IS WRIST JOINT
     **/

    private final float[] linkLengths;
    private final double shortReach;
    private final double[] jointMins;
    private final double[] jointMaxes;
    private final ElapsedTime controllerTimer;
    private final int numberOfJoints;
    private final float verticalOffset;

    public KinematicCalculator (
            int numberOfJoints, float[] linkLengths, double[] jointMins,
            double[] jointMaxes, float verticalOffset
    ) {
        if (jointMins.length != numberOfJoints || jointMaxes.length != numberOfJoints) {
            throw new IllegalArgumentException("Invalid number of joints");
        }
        this.numberOfJoints = numberOfJoints;
        this.linkLengths = linkLengths;
        this.shortReach = linkLengths[0] + linkLengths[1];
        this.jointMins = new double[numberOfJoints];
        this.jointMaxes = new double[numberOfJoints];
        for (int i = 0; i < this.jointMins.length; i++) {
            this.jointMins[i] = Math.toRadians(jointMins[i]);
            this.jointMaxes[i] = Math.toRadians(jointMaxes[i]);
        }
        this.controllerTimer = new ElapsedTime();
        this.verticalOffset = verticalOffset;
    }

    public double[] jointsFromVector(VectorF vector) {
        if (vector.length() != 4) {
            throw new IllegalArgumentException("Vector must be formatted as (x, y, z, gamma)");
        }
        vector.put(2, vector.get(2) - verticalOffset);
        // Calculate joint angles from vector and gamma (pitch) using inverse kinematics
        double baseAngle = Math.atan2(vector.get(1), vector.get(0));
        double baseMagnitude = Math.hypot(vector.get(0), vector.get(1));

        double reachXOffset = Math.cos(vector.get(3)) * linkLengths[2];
        double reachYOffset = Math.sin(vector.get(3)) * linkLengths[2];

        // The 0.99 is there to prevent code from bugging out at edge of reach
        double reach = Math.min(1, 0.99 * shortReach /
                Math.hypot(baseMagnitude - reachXOffset, vector.get(2) - reachYOffset));
        double limitedMagnitude = reachXOffset + (baseMagnitude - reachXOffset) * reach;
        double limitedZ = reachYOffset + (vector.get(2) - reachYOffset) * reach;

        double x3 = limitedMagnitude - linkLengths[2] * Math.cos(baseAngle);
        double y3 = limitedZ - linkLengths[2] * Math.sin(baseAngle);
        double C = Math.hypot(x3, y3);

        double a = Math.acos(
                Math.min(1, Math.max(-1,(Math.pow(linkLengths[0], 2) + Math.pow(linkLengths[1], 2) - Math.pow(C, 2)) /
                (2 * linkLengths[0] * linkLengths[1])))
        );
        double B = Math.acos(
                Math.min(1, Math.max(-1,(Math.pow(linkLengths[0], 2) + Math.pow(C, 2) - Math.pow(linkLengths[1], 2)) /
                (2 * linkLengths[0] * C)))
        );

        double[] jointAngles = new double[4];
        jointAngles[0] = baseAngle;
        jointAngles[1] = Math.atan2(y3, x3) + B;
        jointAngles[2] = -(Math.PI - a);
        jointAngles[3] = vector.get(3) - jointAngles[1] - jointAngles[2];

        for (int i = 0; i < jointAngles.length; i++) {
            jointAngles[i] = Math.max(Math.min(jointAngles[i], jointMaxes[i]), jointMins[i]);
        }
        return jointAngles;
    }

    public VectorF vectorFromJointAngles(double[] jointAngles) {
        if (jointAngles.length != numberOfJoints) {
            throw new IllegalArgumentException("Invalid number of joints");
        }
        // Calculate vector from joint angles
        // Gamma never gets constrained, so don't worry about it

        // This section is planar with the robot arm's theta angle
        double x2 = linkLengths[0] * Math.cos(jointAngles[1]);
        double x3 = x2 + linkLengths[1] * Math.cos(jointAngles[1] + jointAngles[2]);
        double xe = x3 + linkLengths[2] * Math.cos(jointAngles[1] + jointAngles[2] + jointAngles[3]);
        double y2 = linkLengths[0] * Math.sin(jointAngles[1]);
        double y3 = y2 + linkLengths[1] * Math.sin(jointAngles[1] + jointAngles[2]);
        double ye = y3 + linkLengths[2] * Math.sin(jointAngles[1] + jointAngles[2] + jointAngles[3]);

        // This section is the planar part rotated into 3D space
        float x = (float) (xe * Math.cos(jointAngles[0]));
        float y = (float) (xe * Math.sin(jointAngles[0]));
        float z = (float) ye  + verticalOffset;
        float g = (float) (jointAngles[1] + jointAngles[2] + jointAngles[3]);

        return new VectorF(x, y, z, g);
    }

    public void moveVectorWithJoystick(VectorF vector, VectorF joystick,
                                       double moveSpeed, double gammaSpeed)
    {
        if (vector.length() != 4 || joystick.length() != 4) {
            throw new IllegalArgumentException("Vectors must be formatted as (x, y, z, gamma)");
        }

        int msInterval = 10;
        double moveMult = moveSpeed / (1000.0 / msInterval);
        double gammaMult = gammaSpeed / (60000.0 / msInterval);
        if (controllerTimer.milliseconds() > msInterval) {
            for (int i = 0; i < 4; i++) {
                vector.put(i, (float) (vector.get(i) + joystick.get(i) *
                        (i == 3 ? gammaMult : moveMult)));
            }
            controllerTimer.reset();
        }
    }

}
