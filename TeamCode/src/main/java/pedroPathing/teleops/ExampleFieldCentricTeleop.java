package pedroPathing.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example teleop that showcases movement and field-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 12/30/2024
 */
@Config
class speedLimits { // Measured from 0 to 1
    static float armSpeedLimit = 1F;
    static float ascendSpeedLimit = 1F;
    static float wheelServoSpeedLimit = 1F;
    static float ascend2SpeedLimit = 1F;
    static float ascendServo3SpeedLimit = 1F;
    static float armAngleSpeedLimit = 1F;
}
@Config
class currentThresholds { // Measured in milliamps
    static short ascend = 2500;
    static short arm = 2500;
    static short armAngle = 2500;
}

@TeleOp(name = "Example Field-Centric Teleop", group = "Examples")
public class ExampleFieldCentricTeleop extends OpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);
    CRServo wheelServo = null;
    CRServo ascendServo3 = null;
    DcMotorEx armMotor = null;
    DcMotorEx armAngleMotor = null;
    DcMotorEx ascend = null;
    DcMotorEx ascend2 = null;

    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        // Group all motors in an array
        DcMotorEx[] allMiscMotors = new DcMotorEx[] {
                ascend, ascend2, armMotor, armAngleMotor
        };

        ascend = hardwareMap.get(DcMotorEx.class, "ascend");
        ascend2 = hardwareMap.get(DcMotorEx.class, "ascend2");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armAngleMotor = hardwareMap.get(DcMotorEx.class, "armAngleMotor");
        wheelServo = hardwareMap.get(CRServo.class, "wheelServo");
        ascendServo3 = hardwareMap.get(CRServo.class, "ascendServo3");

        for (DcMotorEx motor : allMiscMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.speak("Initialized!");
            telemetry.update();
        }
    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        follower.startTeleopDrive();

        telemetry.speak("Started!");
        telemetry.update();
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {
        ascend.setCurrentAlert(currentThresholds.ascend, CurrentUnit.MILLIAMPS);
        armMotor.setCurrentAlert(currentThresholds.arm, CurrentUnit.MILLIAMPS);
        armAngleMotor.setCurrentAlert(currentThresholds.armAngle, CurrentUnit.MILLIAMPS);

        /* Update Pedro to move the robot based on:
        - Forward/Backward Movement: -gamepad1.left_stick_y
        - Left/Right Movement: -gamepad1.left_stick_x
        - Turn Left/Right Movement: -gamepad1.right_stick_x
        - Robot-Centric Mode: false
        */

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();

        // MISCELLANEOUS MOTOR CONTROLS
        // The getMode in the ifs is so that it only captures the current position once
        // Controls for front lift
        // Better method of handling trigger control
        double armPower = gamepad1.right_trigger - gamepad1.left_trigger;
        if (armPower != 0) {
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setPower(speedLimits.armSpeedLimit * armPower);
        } else if (armMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            // When released, hold at position
            armMotor.setTargetPosition(armMotor.getCurrentPosition());
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(speedLimits.armSpeedLimit);
        }

        // Controls for back lift
        if (gamepad1.a && gamepad1.x) {
            ascend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ascend.setPower(speedLimits.ascendSpeedLimit);
        } else if (gamepad1.a && gamepad1.b) {
            ascend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ascend.setPower(-speedLimits.ascendSpeedLimit);
        } else if (ascend.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            // When released, hold at position
            ascend.setTargetPosition(ascend.getCurrentPosition());
            ascend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ascend.setPower(speedLimits.ascendSpeedLimit);

            // If not comboing buttons, use for wheel servo instead
            if (gamepad1.a) {
                wheelServo.setPower(speedLimits.wheelServoSpeedLimit);
            } else if (gamepad1.y) {
                wheelServo.setPower(-speedLimits.wheelServoSpeedLimit);
            } else {
                // When released, shut off
                wheelServo.setPower(0);
            }
        }

        // Controls for arm angle
        if (gamepad1.left_bumper) {
            armAngleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armAngleMotor.setPower(-speedLimits.armAngleSpeedLimit);
        } else if (gamepad1.right_bumper) {
            armAngleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armAngleMotor.setPower(speedLimits.armAngleSpeedLimit);
        } else if (armAngleMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            // When released, hold at position
            armAngleMotor.setTargetPosition(armAngleMotor.getCurrentPosition());
            armAngleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armAngleMotor.setPower(speedLimits.armAngleSpeedLimit);
        }

        // Controls for winch motor
        if (gamepad1.dpad_up) {
            ascend2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ascend2.setPower(speedLimits.ascend2SpeedLimit);
        } else if (gamepad1.dpad_down) {
            ascend2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ascend2.setPower(-speedLimits.ascend2SpeedLimit);
        } else if (ascend2.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            // When released, hold at position
            ascend2.setTargetPosition(ascend2.getCurrentPosition());
            ascend2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ascend2.setPower(speedLimits.ascend2SpeedLimit);
        }

        // Controls for hook servo
        if (gamepad1.dpad_left) {
            ascendServo3.setPower(speedLimits.ascendServo3SpeedLimit);
        } else if (gamepad1.dpad_right) {
            ascendServo3.setPower(0);
        }

        // MOTOR OVERCURRENT TRIPS

        // Arm motor current trip
        if (armMotor.isOverCurrent()) {
            armMotor.setPower(0);
        } else if (armMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            // Only reset current trip if in RUN_TO_POSITION mode
            armMotor.setPower(speedLimits.armSpeedLimit);
        }

        // Back lift motor current trip
        if (ascend.isOverCurrent() && gamepad1.a && gamepad1.x) {
            ascend.setPower(0);
        } else if (ascend.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            // Only reset current trip if in RUN_TO_POSITION mode
            ascend.setPower(speedLimits.ascendSpeedLimit);
        }

        // Arm angle motor current trip
        if (armAngleMotor.isOverCurrent()) {
            armAngleMotor.setPower(0);
        } else if (armAngleMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            // Only reset current trip if in RUN_TO_POSITION mode
            armAngleMotor.setPower(speedLimits.armAngleSpeedLimit);
        }

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        /* Update Telemetry to the Driver Hub */
        telemetry.update();

    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}