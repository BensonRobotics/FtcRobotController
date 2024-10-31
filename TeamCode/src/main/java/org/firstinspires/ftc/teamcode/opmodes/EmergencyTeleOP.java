package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class
EmergencyTeleOP extends LinearOpMode {

    // TPSmotorRPM = (motorRPM / 60) * motorStepsPerRevolution
    // Output is basically the motor's max speed in encoder steps per second, which is what setVelocity uses
    // 537.7 is a 312 RPM motor's encoder steps per revolution
    public static final double TPS312 = (312.0/60.0) * 537.7;
    public static boolean isLiftHoming = false;
    public static ElapsedTime runtime = new ElapsedTime();
    public static double acceleratedDriveMagnitude = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Declare our devices!! Yayy!!!
        DcMotorEx frontLeftMotor;
        DcMotorEx frontRightMotor;
        DcMotorEx backLeftMotor;
        DcMotorEx backRightMotor;
        DcMotorEx liftMotor;
        CRServo grabberServo;

        // Assign our devices
        // Make sure your ID's match your configuration
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        grabberServo = hardwareMap.get(CRServo.class, "grabberServo");

        // Set lift motor overcurrent amperage
        liftMotor.setCurrentAlert(2000, CurrentUnit.MILLIAMPS);

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up USB forward
        imu.initialize(parameters);

        // Make sure lift doesn't fall under gravity
        // Just a failsafe, as setTargetPosition holds at position anyway
        // ...Unless the lift is joystick-based and not height preset-based
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set drive motors to RUN_USING_ENCODER
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reset drive system motor encoders
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Reset runtime variable, not used yet
        runtime.reset();

        // Play button is pressed
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Get and assign joystick values. remember, Y stick value is reversed
            // These would normally be y, x, and rx, see next comment
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double ry = -gamepad1.right_stick_y;

            // Calculate angle and magnitude from joystick values
            double driveAngle = Math.atan2(x, y);
            double driveMagnitude = Math.hypot(x, y);

            // Cubic root scaling for driveMagnitude and rotation stick, improved control at lower speeds
            // The Math.abs is there so that it maintains its sign and doesn't spit out complex numbers
            // Math.cbrt and Math.sqrt are always faster than using Math.pow
            double scaledDriveMagnitude = Math.cbrt(Math.abs(driveMagnitude)) * driveMagnitude;
            double scaledRX = Math.cbrt(Math.abs(rx)) * rx;
            double scaledRY = Math.cbrt(Math.abs(ry)) * ry;

            // Linear acceleration code for driveMagnitude
            // +0.05 to -0.05 is the tolerance where it will just bypass the incrementer to reduce wobbling around the target speed
            // If acceleration is too fast, decrease the increment values
            if (acceleratedDriveMagnitude - scaledDriveMagnitude < -0.05) {
                acceleratedDriveMagnitude = acceleratedDriveMagnitude + 0.01;
            } else if (acceleratedDriveMagnitude - scaledDriveMagnitude > 0.05) {
                acceleratedDriveMagnitude = acceleratedDriveMagnitude - 0.01;
            } else {
                acceleratedDriveMagnitude = scaledDriveMagnitude;
            }

            // IMU Yaw reset button
            // This button choice was made so that it is hard to hit on accident
            if (gamepad1.back) {
                imu.resetYaw();
            }
            // Set botHeading to robot Yaw from IMU
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            // The evil code for calculating motor powers
            // Desmos used to troubleshoot directions without robot
            // https://www.desmos.com/calculator/3gzff5bzbn
            double frontLeftBackRightMotors = acceleratedDriveMagnitude * Math.sin(driveAngle + botHeading + 0.25 * Math.PI);
            double frontRightBackLeftMotors = acceleratedDriveMagnitude * -Math.sin(driveAngle + botHeading - 0.25 * Math.PI);
            double frontLeftPower = frontLeftBackRightMotors + scaledRX;
            double backLeftPower = frontRightBackLeftMotors + scaledRX;
            double frontRightPower = frontRightBackLeftMotors - scaledRX;
            double backRightPower = frontLeftBackRightMotors - scaledRX;

            // The Great Cleaving approaches
            // Forgive me for what I'm about to do, I took a melatonin an hour ago and I want to collapse onto my bed at this point
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio
            double denominator = Math.max(Math.max(Math.max(Math.abs(frontRightPower),Math.abs(frontLeftPower)),Math.max(Math.abs(backRightPower),Math.abs(backLeftPower))),1);
            // CLEAVING TIME
            frontRightPower /= denominator;
            frontLeftPower /= denominator;
            backRightPower /= denominator;
            backLeftPower /= denominator;

            // Set motor velocities, converted from (-1 to 1) to (-TPS312 to TPS312)
            frontLeftMotor.setVelocity(frontLeftPower * TPS312);
            backLeftMotor.setVelocity(backLeftPower * TPS312);
            frontRightMotor.setVelocity(frontRightPower * TPS312);
            backRightMotor.setVelocity(backRightPower * TPS312);

            /*
            // Lift motor position-based code starts here

            // Lift homing button
            if (gamepad1.start) {
                isLiftHoming = true;
            }
            // Lift motor height presets, if not homing
            // A for bottom, X for middle, Y for top
            if (!isLiftHoming) {
                if (gamepad1.a) {
                    liftMotor.setTargetPosition(0);
                }
                if (gamepad1.x) {
                    liftMotor.setTargetPosition(3000);
                }
                if (gamepad1.y) {
                    liftMotor.setTargetPosition(4300);
                }
                // Tell liftMotor to run to to target position at set speed
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor.setPower(0.75);

            } else { // If liftMotor is in fact homing
                liftMotor.setCurrentAlert(1500, CurrentUnit.MILLIAMPS);
                if (!liftMotor.isOverCurrent()) {
                    liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    liftMotor.setPower(-0.25);
                } else { // Once homing is finished
                    liftMotor.setPower(0);
                    liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    liftMotor.setTargetPosition(0);
                    liftMotor.setCurrentAlert(3000, CurrentUnit.MILLIAMPS);
                    isLiftHoming = false;
                }

            }
            // Lift motor position-based code ends here
             */

            // Other lift motor code. Insanely complicated.
            // Lift is controlled by right stick Y axis
            liftMotor.setVelocity(scaledRY*TPS312);

            // Grabber servo code. Super complicated.
            // If you let go of right bumper, servo will stay running forward at a lower speed
            // If you let go of left bumper, servo will stop
            if (gamepad1.right_bumper) {
                grabberServo.setPower(1);
            } else if (grabberServo.getPower()>0){
                grabberServo.setPower(0.25);
            }
            if (gamepad1.left_bumper) {
                grabberServo.setPower(-1);
            } else if (grabberServo.getPower()<0) {
                grabberServo.setPower(0);
            }

            // If liftMotor overcurrents, stop it
            if (liftMotor.isOverCurrent()) {
                liftMotor.setPower(0);
            }

            // Telemetry
            // Telemetry related to position-based lift motor code is commented out as well
            /* telemetry.addData("Lift Height:",liftMotor.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Lift Target:",liftMotor.getTargetPosition());
            telemetry.addLine(); */
            telemetry.addData("Lift Current (Milliamps):",liftMotor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addLine();
            telemetry.addData("Lift Homing:",isLiftHoming);

            telemetry.update();
        }
    }
}