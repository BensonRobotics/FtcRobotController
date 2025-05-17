package org.firstinspires.ftc.teamcode.robotarm;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

import au.edu.federation.utils.Vec3f;

@TeleOp(name = "Robot Arm 3D Test", group = "Test")
public class RobotArmTest3D extends OpMode {

    // Initial end effector position, it will limit to be within reach on first loop
    private Vec3f endEffector;
    private double gamma = 0;
    private static final int MAX_MOVE_SPEED = 100; // Measured in mm/s
    private static final int MAX_GAMMA_SPEED = 15; // Measured in RPM
    private static final int CONTROL_INTERVAL = 50; // Measured in ms
    Controller controller;
    CalikoCalc calculator;
    private final ElapsedTime controllerTimer = new ElapsedTime();

    @Override
    public void init() {
        controllerTimer.reset();
        controller = new Controller(hardwareMap);
        calculator = new CalikoCalc();
        endEffector = calculator.getEndEffector();
        updateTelemetry();
    }

    @Override
    public void loop() {
        double gammaJoystick = (gamepad1.dpad_up) ? 1 : ((gamepad1.dpad_down) ? -1 : 0);
        Vec3f joystick = new Vec3f(gamepad1.left_stick_x, -gamepad1.left_stick_y,
                -gamepad1.right_stick_y);

        if (!joystick.lengthIsApproximately(0, 0.05f)) {
            if (controllerTimer.milliseconds() > CONTROL_INTERVAL) {
                // Move vector with joystick
                endEffector = endEffector.plus(joystick.times(
                        (float) (MAX_MOVE_SPEED / (1000.0 / CONTROL_INTERVAL))));
                gamma += gammaJoystick * (MAX_GAMMA_SPEED / (60000.0 / CONTROL_INTERVAL));

                calculator.setTarget(endEffector);

                controllerTimer.reset();
            }
        } else if (endEffector != calculator.getEndEffector()) {
            endEffector = calculator.getEndEffector();
        }

        // Apply constrained joint angles to motors
        controller.applyJointAngles(calculator.getUnwrappedJointAngles());

        // Set vector from constrained joint angles
        endEffector = calculator.getEndEffector();

        updateTelemetry();
    }

    private void updateTelemetry() {
        telemetry.addData("Raw Joint Angles", Arrays.toString(calculator.getRawJointAngles()));
        telemetry.addData("Unwrapped Joint Angles",
                Arrays.toString(calculator.getUnwrappedJointAngles()));
        telemetry.addData("End Effector Position", endEffector.toString());
        telemetry.update();
    }
}
