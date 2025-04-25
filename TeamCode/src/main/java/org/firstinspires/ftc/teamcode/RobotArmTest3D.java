package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

@TeleOp(name = "Robot Arm 2D Test", group = "Test")
public class RobotArmTest3D extends OpMode {

    private final float[] L = {24*13, 9*24, 11*24};
    private VectorF endEffector = new VectorF(0, 0, 0, 0);
    private final double[] Jmax = {180, 95, 170, 300/2.0};
    private final double[] Jmin = {0, -95, -170, -300/2.0};
    private final String[] jointNames = {"base", "shoulder", "elbow", "wrist"};
    private final double[] motorPPROrServoRange = {1000, 5281.1, 2786.2, 300};
    KineCalc3D calculator;
    JointController jointController;

    @Override
    public void init() {
        calculator.initialize(L, Jmin, Jmax);
        jointController.initialize(hardwareMap, jointNames, motorPPROrServoRange);
    }

    @Override
    public void loop() {
        double gammaJoystick = (gamepad1.dpad_up) ? 1 : ((gamepad1.dpad_down) ? -1 : 0);
        VectorF joystick = new VectorF(gamepad1.left_stick_x, -gamepad1.left_stick_y,
                -gamepad1.right_stick_y, (float) gammaJoystick);
        // Measured in mm/s
        int maxMoveSpeed = 100;
        // Measured in RPM
        int maxGammaSpeed = 15;
        calculator.moveVectorWithJoystick(endEffector, joystick, maxMoveSpeed, maxGammaSpeed);

        // Calculate joint angles from vector and gamma (pitch)
        double[] J = calculator.jointsFromVector(endEffector);

        // Apply constrained joint angles to motors
            jointController.applyJointAngles(J);

            // Leave this off, it could cause feedback issues
            //J = jointController.readJointAngles();

        // Set vector from constrained joint angles
        endEffector = calculator.vectorFromJointAngles(J);
    }
}
