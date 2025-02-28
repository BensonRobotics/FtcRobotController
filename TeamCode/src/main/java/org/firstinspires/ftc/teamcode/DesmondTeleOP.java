// Package
package org.firstinspires.ftc.teamcode;

// Imports
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

/*
Controller mapping goes HERE:


 */

// Class
@TeleOp
public class
DesmondTeleOP extends LinearOpMode {
    final byte hoursYouHaveLeft = 12;

    // Drive system PIDF coefficients
    final float NEW_P_DRIVE = 0.75F;
    final float NEW_I_DRIVE = 0.2F;
    final float NEW_D_DRIVE = 0.1F;
    final float NEW_F_DRIVE = 10.0F;

    // driveTicksPerSecond = driveMotorRPM * driveMotorStepsPerRevolution / 60
    // Output is basically the motor's max speed in encoder steps per second, which is what setVelocity uses
    // 537.7 is a 312 RPM motor's encoder steps per revolution
    // Output is first cast to float, since the equation itself uses double precision
    final float driveTicksPerSecond = (float) (312.0 * 537.7 / 60.0);
    ElapsedTime runtime = new ElapsedTime();
    final boolean useFieldCentricDrive = true;
    final short ascendCurrentAlert = 2500;
    final short armCurrentAlert = 2500;
    final short armAngleCurrentAlert = 2500;
    final float driveSpeedLimit = 1F;
    final float ascendSpeedLimit = 1F;
    final float armSpeedLimit = 1F;
    final float armAngleSpeedLimit = 1F;
    final float ascend2SpeedLimit = 1F;
    final float ascendServo3SpeedLimit = 1F;
    final float wheelServoSpeedLimit = 1F;
    double driveHeading = 0; // If field centric is disabled, this will always stay at 0
    double rotationPower;

    // Declare our devices!! Yayy!!!
    DcMotorEx frontLeftMotor;
    DcMotorEx frontRightMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx backRightMotor;
    CRServo wheelServo;
    CRServo ascendServo3;
    DcMotorEx armMotor;
    DcMotorEx armAngleMotor;
    DcMotorEx ascend;
    DcMotorEx ascend2;

    @Override
    public void runOpMode() {

        // Assign our devices
        // Make sure your ID's match your configuration
        wheelServo = hardwareMap.get(CRServo.class, "wheelServo");
        ascendServo3 = hardwareMap.get(CRServo.class, "ascendServo3");
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "back_right_drive");
        ascend = hardwareMap.get(DcMotorEx.class, "ascend");
        ascend2 = hardwareMap.get(DcMotorEx.class, "ascend2");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armAngleMotor = hardwareMap.get(DcMotorEx.class, "armAngleMotor");

        // Group drive motors in an array
        DcMotorEx[] driveMotors = new DcMotorEx[] {
                frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor
        };

        // Group all motors in an array
        DcMotorEx[] allMotors = new DcMotorEx[] {
                frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, ascend, ascend2, armMotor, armAngleMotor
        };

        // Set PIDF values for all drive motors
        for (DcMotorEx motor : driveMotors) {
            // Apply drive PIDF coefficients
            motor.setVelocityPIDFCoefficients(NEW_P_DRIVE,NEW_I_DRIVE,NEW_D_DRIVE,NEW_F_DRIVE);
        }

        // Reverse motors that are otherwise backwards
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        // Without this, the REV Hub's orientation is assumed to be logo up USB forward
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        // Set all motors to brake mode, reset encoders, and run using encoders
        for (DcMotorEx motor : allMotors) {
            // For example, setting brake mode and encoder run mode:
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Set motor current thresholds
        ascend.setCurrentAlert(ascendCurrentAlert, CurrentUnit.MILLIAMPS);
        armMotor.setCurrentAlert(armCurrentAlert, CurrentUnit.MILLIAMPS);
        armAngleMotor.setCurrentAlert(armAngleCurrentAlert, CurrentUnit.MILLIAMPS);

        // Reset runtime, fully initialized!
        runtime.reset();
        telemetry.speak("Initialized!");
        telemetry.update();

        // Play button is pressed
        waitForStart();

        // OpMode started!
        telemetry.speak("Started!");
        telemetry.update();

        if (isStopRequested()) return;

        // Main loop
        while (opModeIsActive()) {
            // Get and assign joystick values. remember, Y stick value is reversed
            // These would normally be y, x, and rx, see next comment
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double ry = -gamepad1.right_stick_y;

            // Get IMU yaw value
            double botHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Calculate angle and magnitude from joystick values
            double driveAngle = Math.atan2(x, y);
            double driveMagnitude = Math.hypot(x, y);

            // IMU Yaw reset button
            // This button choice was made so that it is hard to hit on accident
            if (gamepad1.back) {
                imu.resetYaw();
            }
            // Set botHeading to robot Yaw from IMU, if used
            // If otherwise, driveAngle will stay at zero, effectively disabling field centric
            if (useFieldCentricDrive) {
                driveHeading = botHeading;
            }

            // Calculate rotation power from right stick
            rotationPower = rx * driveSpeedLimit;

            // The evil code for calculating motor powers
            // Desmos used to troubleshoot directions without robot
            // https://www.desmos.com/calculator/3gzff5bzbn
            double frontLeftBackRightMotors = driveSpeedLimit * driveMagnitude * Math.sin(driveAngle - driveHeading + 0.25 * Math.PI);
            double frontRightBackLeftMotors = driveSpeedLimit * driveMagnitude * -Math.sin(driveAngle - driveHeading - 0.25 * Math.PI);
            double frontLeftPower = frontLeftBackRightMotors + rotationPower;
            double backLeftPower = frontRightBackLeftMotors + rotationPower;
            double frontRightPower = frontRightBackLeftMotors - rotationPower;
            double backRightPower = frontLeftBackRightMotors - rotationPower;

            // The Great Cleaving approaches
            // Forgive me for what I'm about to do, I took a melatonin an hour ago and I want to collapse onto my bed at this point
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio
            double denominator = Math.max(Math.max(Math.max(Math.abs(frontRightPower),Math.abs(frontLeftPower)),
                    Math.max(Math.abs(backRightPower),Math.abs(backLeftPower))),1);
            // CLEAVING TIME
            frontRightPower /= denominator;
            frontLeftPower /= denominator;
            backRightPower /= denominator;
            backLeftPower /= denominator;

            // Set motor velocities, converted from (-1 to 1) to (-driveTicksPerSecond to driveTicksPerSecond)
            frontLeftMotor.setVelocity(frontLeftPower * driveTicksPerSecond);
            backLeftMotor.setVelocity(backLeftPower * driveTicksPerSecond);
            frontRightMotor.setVelocity(frontRightPower * driveTicksPerSecond);
            backRightMotor.setVelocity(backRightPower * driveTicksPerSecond);

            // MISCELLANEOUS MOTOR CONTROLS
            // The getMode in the ifs is so that it only captures the current position once
            // Controls for front lift
            // Better method of handling trigger control
            double armPower = gamepad1.right_trigger - gamepad1.left_trigger;
            if (armPower != 0) {
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armMotor.setPower(armSpeedLimit * armPower);
            } else if (armMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                // When released, hold at position
                armMotor.setTargetPosition(armMotor.getCurrentPosition());
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(armSpeedLimit);
            }

            // Controls for back lift
            if (gamepad1.a && gamepad1.x) {
                wheelServo.setPower(0);
                ascend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ascend.setPower(ascendSpeedLimit);
            } else if (gamepad1.a && gamepad1.b) {
                wheelServo.setPower(0);
                ascend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ascend.setPower(-ascendSpeedLimit);
            } else {
                if (ascend.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                    // When released, hold at position
                    ascend.setTargetPosition(ascend.getCurrentPosition());
                    ascend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ascend.setPower(ascendSpeedLimit);
                }

                // If not comboing buttons, use for wheel servo instead
                if (gamepad1.a) {
                    wheelServo.setPower(-wheelServoSpeedLimit);
                } else if (gamepad1.y) {
                    wheelServo.setPower(wheelServoSpeedLimit);
                } else {
                    // When released, shut off
                    wheelServo.setPower(0);
                }
            }

            // Controls for arm angle
            if (gamepad1.left_bumper) {
                armAngleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armAngleMotor.setPower(-armAngleSpeedLimit);
            } else if (gamepad1.right_bumper) {
                armAngleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armAngleMotor.setPower(armAngleSpeedLimit);
            } else if (armAngleMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                // When released, hold at position
                armAngleMotor.setTargetPosition(armAngleMotor.getCurrentPosition());
                armAngleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armAngleMotor.setPower(armAngleSpeedLimit);
            }

            // Controls for winch motor
            if (gamepad1.dpad_up) {
                ascend2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ascend2.setPower(ascend2SpeedLimit);
            } else if (gamepad1.dpad_down) {
                ascend2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ascend2.setPower(-ascend2SpeedLimit);
            } else if (ascend2.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                // When released, hold at position
                ascend2.setTargetPosition(ascend2.getCurrentPosition());
                ascend2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ascend2.setPower(ascend2SpeedLimit);
            }

            // Controls for hook servo
            if (gamepad1.dpad_left) {
                ascendServo3.setPower(ascendServo3SpeedLimit);
            } else if (gamepad1.dpad_right) {
                ascendServo3.setPower(0);
            }

            // MOTOR OVERCURRENT TRIPS

            // Arm motor current trip
            if (armMotor.isOverCurrent()) {
                armMotor.setPower(0);
            } else if (armMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                // Only reset current trip if in RUN_TO_POSITION mode
                armMotor.setPower(armSpeedLimit);
            }

            // Back lift motor current trip
            if (ascend.isOverCurrent() && gamepad1.a && gamepad1.x) {
                ascend.setPower(0);
            } else if (ascend.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                // Only reset current trip if in RUN_TO_POSITION mode
                ascend.setPower(ascendSpeedLimit);
            }

            // Arm angle motor current trip
            if (armAngleMotor.isOverCurrent()) {
                armAngleMotor.setPower(0);
            } else if (armAngleMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                // Only reset current trip if in RUN_TO_POSITION mode
                armAngleMotor.setPower(armAngleSpeedLimit);
            }

            telemetry.addData("botHeading", botHeading);
            telemetry.addData("armAngleMotor.getCurrentPosition()", armAngleMotor.getCurrentPosition());
            telemetry.addData("armMotor.getCurrentPosition()", armMotor.getCurrentPosition());
            telemetry.addData("ascend.getCurrentPosition()", ascend.getCurrentPosition());
            telemetry.update();
        }
    }
}