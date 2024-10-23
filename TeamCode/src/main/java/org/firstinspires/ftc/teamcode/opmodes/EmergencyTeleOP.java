package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class EmergencyTeleOP extends LinearOpMode {

    // PIDF coefficients for drive system's setVelocity
    public static final double NEW_P_DRIVE = 1.5;
    public static final double NEW_I_DRIVE = 0.2;
    public static final double NEW_D_DRIVE = 0.1;
    public static final double NEW_F_DRIVE = 12.0;

    //PIDF coefficients for lift motor's setPosition
    public static final double NEW_P_LIFT = 1.5;
    public static final double NEW_I_LIFT = 0.2;
    public static final double NEW_D_LIFT = 0.1;
    public static final double NEW_F_LIFT = 12.0;

    // TPS(motorRPM) = (motorRPM / 60) * motorStepsPerRevolution
    // Output is basically the motor's max speed in encoder steps per second, which is what setVelocity uses
    // 537.7 is a 312 RPM motor's encoder steps per revolution
    public static final double TPS312 = (312.0/60.0) * 537.7;
    public static ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx frontLeftMotor;
        DcMotorEx frontRightMotor;
        DcMotorEx backLeftMotor;
        DcMotorEx backRightMotor;
        DcMotorEx liftMotor;
        Servo grabberServo;

        // Declare our motors
        // Make sure your ID's match your configuration
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        grabberServo = hardwareMap.get(Servo.class, "grabberServo");

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
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        // New variable for drive system's PIDF coefficients
        PIDFCoefficients pidfNewDrive = new PIDFCoefficients(NEW_P_DRIVE, NEW_I_DRIVE, NEW_D_DRIVE, NEW_F_DRIVE);
        PIDFCoefficients pidfNewLift = new PIDFCoefficients(NEW_P_LIFT, NEW_I_LIFT, NEW_D_LIFT, NEW_F_LIFT);

        // Sets PIDF coefficients for drive system and lift motor using variables
        frontRightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNewDrive);
        frontLeftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNewDrive);
        backRightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNewDrive);
        backLeftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNewDrive);
        liftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfNewLift);

        // Make sure lift doesn't fall under gravity
        // Just a failsafe, as setTargetPosition holds at position anyway
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset drive system motor encoders
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        runtime.reset();

        // Lift sensorless homing code, will move the lift during initialization
        // Using built-in CurrentAlert is easier
        liftMotor.setCurrentAlert(3000, CurrentUnit.MILLIAMPS);
        while (!liftMotor.isOverCurrent()) {
            liftMotor.setVelocity(-0.1 * TPS312);
        }
        liftMotor.setVelocity(0);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Play button is pressed
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Cubic root scaling for joysticks, for improved control at lower speeds
            // The Math.abs is there so that it maintains its sign and doesn't spit out complex numbers
            // Math.cbrt and Math.sqrt are always faster than using Math.pow
            double scaledY = Math.cbrt(Math.abs(y)) * y;
            double scaledX = Math.cbrt(Math.abs(x)) * x;
            double scaledRx = Math.cbrt(Math.abs(rx)) * rx;

            // Factor to correct for imperfect strafing
            scaledX = scaledX * 1.1;

            double driveAngle = Math.atan2(scaledX,scaledY);
            double driveMagnitude = Math.hypot(scaledX,scaledY);

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            if (gamepad1.back) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double frontLeftBackRightMotors = Math.sin(driveAngle - botHeading + 0.25 * Math.PI);
            double frontRightBackLeftMotors = Math.sin(driveAngle - botHeading - 0.25 * Math.PI);
            double frontLeftPower = frontLeftBackRightMotors - scaledRx;
            double backLeftPower = frontRightBackLeftMotors - scaledRx;
            double frontRightPower = frontRightBackLeftMotors + scaledRx;
            double backRightPower = frontLeftBackRightMotors + scaledRx;

            // Forgive me for what I'm about to do, I took a melatonin an hour ago and I want to collapse onto my bed at this point
            // Denominator is the largest motor power (absolute value)
            // This ensures all the powers maintain the same ratio
            double denominator = Math.max(Math.max(Math.abs(frontRightPower),Math.abs(frontLeftPower)),Math.max(Math.abs(backRightPower),Math.abs(backLeftPower)));
            frontRightPower /= denominator;
            frontLeftPower /= denominator;
            backRightPower /= denominator;
            backLeftPower /= denominator;

            // Set motor velocities, converted from (-1 to 1) to (-TPS312 to TPS312)
            frontLeftMotor.setVelocity(frontLeftPower * driveMagnitude * TPS312);
            backLeftMotor.setVelocity(backLeftPower * driveMagnitude * TPS312);
            frontRightMotor.setVelocity(frontRightPower * driveMagnitude * TPS312);
            backRightMotor.setVelocity(backRightPower * driveMagnitude * TPS312);

            // Lift motor height presets
            // A for bottom, X for middle, Y for top
            if (gamepad1.a) {
                liftMotor.setTargetPosition(0);
            }
            if (gamepad1.x) {
                liftMotor.setTargetPosition(2800);
            }
            if (gamepad1.y) {
                liftMotor.setTargetPosition(4500);
            }

            // If liftMotor overcurrents, stop it
            // Would make this zero the encoder if it hits the bottom,
            // But in practice, it could hit something going down and zero itself at the wrong height
            if (liftMotor.isOverCurrent()) {
                liftMotor.setVelocity(0);
            }
        }
    }
}