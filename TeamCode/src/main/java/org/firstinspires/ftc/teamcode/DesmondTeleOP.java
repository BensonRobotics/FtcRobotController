package org.firstinspires.ftc.teamcode;

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
DesmondTeleOP extends LinearOpMode {
    final byte hoursYouHaveLeft = 12;

    // Drive system PIDF coefficients
    float NEW_P_DRIVE = 0.75F;
    float NEW_I_DRIVE = 0.2F;
    float NEW_D_DRIVE = 0.1F;
    float NEW_F_DRIVE = 10.0F;

    // driveTicksPerSecond = driveMotorRPM * driveMotorStepsPerRevolution / 60
    // Output is basically the motor's max speed in encoder steps per second, which is what setVelocity uses
    // 537.7 is a 312 RPM motor's encoder steps per revolution
    // Output is first cast to float, since the equation itself uses double precision
    float driveTicksPerSecond = (float) (312.0 * 537.7 / 60.0);
    ElapsedTime runtime = new ElapsedTime();
    boolean useFieldCentricDrive = true;
    short ascendCurrentAlert = 2500;
    short armCurrentAlert = 2500;
    float driveSpeedLimit = 1F;
    float ascendSpeedLimit = 1F;
    float armSpeedLimit = 1F;
    float armAngleSpeedLimit = 1F;
    float ascend3SpeedLimit = 1F;
    double driveHeading = 0;
    double rotationPower = 0;

    @Override
    public void runOpMode() {

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
        DcMotorEx ascend3;

        // Assign our devices
        // Make sure your ID's match your configuration
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "back_right_drive");
        wheelServo = hardwareMap.get(CRServo.class, "wheelServo");
        ascendServo3 = hardwareMap.get(CRServo.class, "ascendServo3");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armAngleMotor = hardwareMap.get(DcMotorEx.class, "armAngleMotor");
        ascend = hardwareMap.get(DcMotorEx.class, "ascend");
        ascend3 = hardwareMap.get(DcMotorEx.class, "ascend3");

        // Apply motor PIDF coefficients
        frontLeftMotor.setVelocityPIDFCoefficients(NEW_P_DRIVE,NEW_I_DRIVE,NEW_D_DRIVE,NEW_F_DRIVE);
        frontRightMotor.setVelocityPIDFCoefficients(NEW_P_DRIVE,NEW_I_DRIVE,NEW_D_DRIVE,NEW_F_DRIVE);
        backLeftMotor.setVelocityPIDFCoefficients(NEW_P_DRIVE,NEW_I_DRIVE,NEW_D_DRIVE,NEW_F_DRIVE);
        backRightMotor.setVelocityPIDFCoefficients(NEW_P_DRIVE,NEW_I_DRIVE,NEW_D_DRIVE,NEW_F_DRIVE);

        // Reverse motors that are backwards otherwise
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        ascend.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up USB forward
        imu.initialize(parameters);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ascend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ascend3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armAngleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset drive system motor encoders
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascend3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armAngleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set drive motors to RUN_USING_ENCODER
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ascend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ascend3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armAngleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set lift motor current trip
        ascend.setCurrentAlert(ascendCurrentAlert, CurrentUnit.MILLIAMPS);
        armMotor.setCurrentAlert(ascendCurrentAlert, CurrentUnit.MILLIAMPS);
        armAngleMotor.setCurrentAlert(ascendCurrentAlert, CurrentUnit.MILLIAMPS);

        // Reset runtime
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
            if (useFieldCentricDrive) {
                driveHeading = botHeading;
            }
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

                    // Engages RUN_TO_POSITION when lift stops moving for active position holding
                    double armPower = gamepad1.right_trigger - gamepad1.left_trigger;
                    if (armPower != 0) {
                        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        armMotor.setPower(armSpeedLimit * armPower);
                    } else if (armMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                        armMotor.setTargetPosition(armMotor.getCurrentPosition());
                        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armMotor.setPower(armSpeedLimit);
                    }

                    if (gamepad1.a && gamepad1.x) {
                        ascend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        ascend.setPower(ascendSpeedLimit);
                    } else if (gamepad1.a && gamepad1.b) {
                        ascend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        ascend.setPower(-ascendSpeedLimit);
                    } else if (ascend.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                        ascend.setTargetPosition(ascend.getCurrentPosition());
                        ascend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ascend.setPower(ascendSpeedLimit);

                        if (gamepad1.a) {
                            wheelServo.setPower(1);
                        } else if (gamepad1.y) {
                            wheelServo.setPower(-1);
                        } else {
                            wheelServo.setPower(0);
                        }
                    }

                    if (gamepad1.left_bumper) {
                        armAngleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        armAngleMotor.setPower(-1);
                    } else if (gamepad1.right_bumper) {
                        armAngleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        armAngleMotor.setPower(1);
                    } else if (armAngleMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                        armAngleMotor.setTargetPosition(armAngleMotor.getCurrentPosition());
                        armAngleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armAngleMotor.setPower(1);
                    }

                    if (gamepad1.dpad_up) {
                        ascend3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        ascend3.setPower(1);
                    } else if (gamepad1.dpad_down) {
                        ascend3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        ascend3.setPower(-1);
                    } else if (ascend3.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                        ascend3.setTargetPosition(ascend3.getCurrentPosition());
                        ascend3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ascend3.setPower(1);
                    }

                    if (gamepad1.dpad_left) {
                        ascendServo3.setPower(1);
                    } else if (gamepad1.dpad_right) {
                        ascendServo3.setPower(0);
                    }

                    // Lift motor current trip, only if going up, to allow for potential hanging
                    // Remember to tighten the belt on the lift to prevent skipping
                    if (armMotor.isOverCurrent()) {
                        armMotor.setPower(0);
                    } else if (armMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                        armMotor.setPower(1);
                    }

                    if (ascend.isOverCurrent() && gamepad1.a && gamepad1.x) {
                        ascend.setPower(0);
                    } else if (ascend.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                        ascend.setPower(1);
                    }

                    if (armAngleMotor.isOverCurrent()) {
                        armAngleMotor.setPower(0);
                    } else if (armAngleMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                        armAngleMotor.setPower(1);
                    }

            // Telemetry
            telemetry.addData("Ascend Height:",ascend.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Ascend Target:",ascend.getTargetPosition());
            telemetry.addLine();
            telemetry.addData("Ascend Current:",ascend.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addLine();
            telemetry.update();
        }
    }
}