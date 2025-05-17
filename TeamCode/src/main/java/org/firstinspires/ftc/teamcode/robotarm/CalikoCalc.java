package org.firstinspires.ftc.teamcode.robotarm;

import java.util.Arrays;

import au.edu.federation.caliko.FabrikBone3D;
import au.edu.federation.caliko.FabrikChain3D;
import au.edu.federation.caliko.FabrikJoint3D;
import au.edu.federation.utils.Vec3f;

public class CalikoCalc {
    private final FabrikChain3D chain;
    private final double[] lastJointAngles;
    private final int[] jointWrapCounts;
    public CalikoCalc() {
        chain = new FabrikChain3D();
        lastJointAngles = new double[Config.NUM_OF_JOINTS];
        jointWrapCounts = new int[Config.NUM_OF_JOINTS];

        Arrays.fill(lastJointAngles, 0);
        Arrays.fill(jointWrapCounts, 0);

        Vec3f baseStart = Config.BASE_POINT;
        Vec3f baseEnd = baseStart.plus(Config.INIT_JOINT_DIR.times(Config.BONE_LENGTHS[0]));
        FabrikBone3D baseBone = new FabrikBone3D(baseStart, baseEnd);
        chain.addBone(baseBone);
        chain.setRotorBaseboneConstraint(FabrikChain3D.BaseboneConstraintType3D.GLOBAL_ROTOR,
                Config.INIT_JOINT_DIR, Config.JOINT_CW_LIMITS[1]);

        for (int i = 1; i < Config.BONE_LENGTHS.length; i++) {
            chain.addConsecutiveHingedBone(Config.INIT_JOINT_DIR, Config.BONE_LENGTHS[i],
                    FabrikJoint3D.JointType.LOCAL_HINGE, Config.HINGE_AXIS,
                    Config.JOINT_CW_LIMITS[i+1], Config.JOINT_CCW_LIMITS[i+1],
                    Config.INIT_JOINT_DIR);
        }
    }

    public void setTarget(Vec3f target) { this.chain.solveForTarget(target); }

    public double[] getRawJointAngles() {
        double[] jointAngles = new double[Config.NUM_OF_JOINTS];
        jointAngles[0] = Math.toRadians(this.chain.getBone(0).getGlobalYawDegs());
        jointAngles[1] = Math.toRadians(this.chain.getBone(0).getGlobalPitchDegs());
        for (int i = 1; i < this.chain.getNumBones(); i++) {
            Vec3f p = this.chain.getBone(i-1).getDirectionUV();
            Vec3f c = this.chain.getBone(i).getDirectionUV();
            Vec3f k = Vec3f.rotateAboutAxisRads(Config.HINGE_AXIS, (float) jointAngles[0], Config.INIT_JOINT_DIR);
            double angle = Math.toRadians(Vec3f.getSignedAngleBetweenDegs(p, c, k));
            jointAngles[i+1] = angle;
        }
        for (int i = 0; i < jointAngles.length; i++) {
            double diff = jointAngles[i] - this.lastJointAngles[i];
            if (diff < -Math.PI) {
                this.jointWrapCounts[i]++;
            } else if (diff > Math.PI) {
                this.jointWrapCounts[i]--;
            }
        }
        System.arraycopy(jointAngles, 0, this.lastJointAngles, 0, jointAngles.length);

        return jointAngles;
    }

    public double[] getUnwrappedJointAngles() {
        double[] jointAngles = this.getRawJointAngles();
        for (int i = 0; i < jointAngles.length; i++) {
            jointAngles[i] += jointWrapCounts[i] * 2*Math.PI;
            jointAngles[i] += Math.toRadians(Config.JOINT_OFFSETS[i]);
        }
        return jointAngles;
    }

    public Vec3f getEndEffector() { return this.chain.getEffectorLocation(); }
}
