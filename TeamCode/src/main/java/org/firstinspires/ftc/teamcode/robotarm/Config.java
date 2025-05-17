package org.firstinspires.ftc.teamcode.robotarm;

import au.edu.federation.utils.Vec3f;

class Config {
    private Config() {}

    /**
     * 3D space is set up in Cartesian coordinates
     * Positive X is to the right, positive Y is forward, positive Z is up
     * I don't like the Cartesian setups with Y being vertical, it just feels wrong
     */

    public static final int NUM_OF_JOINTS = 4;
    public static final float[] BONE_LENGTHS = {13*24, 11*24, 6*24};
    public static final int[] JOINT_CW_LIMITS = {90, 100, 170, 150}; // Both are positive degs
    public static final int[] JOINT_CCW_LIMITS = {90, 100, 170, 150}; // Both are positive degs
    public static final float[] JOINT_OFFSETS = {-180, -180, 0, 0}; // Degrees
    public static final String[] JOINT_NAMES = {"base", "shoulder", "elbow", "wrist"};
    public static final boolean[] JOINTS_REVERSED = {false, false, true, false};
    public static final float[] JOINT_MOTOR_PPRS = {537.7f*8, 5281.1f, 2786.2f, 0.0f};
    public static final float[] JOINT_SERVO_RANGES = {0.0f, 0.0f, 0.0f, 300.0f};
    public static final Vec3f BASE_POINT = new Vec3f(0.0f, 0.0f, 139.0f);
    public static final Vec3f INIT_JOINT_DIR = new Vec3f(0.0f, 0.0f, 1.0f);
    public static final Vec3f HINGE_AXIS = new Vec3f(1.0f, 0.0f, 0.0f);
    public static final float[] JOINT_MOTOR_PROPORTIONALS = {2.0f, 4.0f, 4.0f, 4.0f};

}
