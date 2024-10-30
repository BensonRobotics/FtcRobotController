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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class EmergencyTeleOP extends LinearOpMode {

    // PIDF stands for Proportional, Integral, Derivative, Feedforward
    // PIDF coefficients for drive system's setVelocity
    public static final double NEW_P_DRIVE = 0.2;
    public static final double NEW_I_DRIVE = 0.1;
    public static final double NEW_D_DRIVE = 0.1;
    public static final double NEW_F_DRIVE = 10.0;

    // PIDF coefficients for lift motor's setVelocity
    // Proportional coefficient for lift motor's setPosition
    public static final double NEW_POS_P_LIFT = 2.0;
    public static final double NEW_P_LIFT = 1.0;
    public static final double NEW_I_LIFT = 0.2;
    public static final double NEW_D_LIFT = 0.1;
    public static final double NEW_F_LIFT = 10.0;

    // TPSmotorRPM = (motorRPM / 60) * motorStepsPerRevolution
    // Output is basically the motor's max speed in encoder steps per second, which is what setVelocity uses
    // 537.7 is a 312 RPM motor's encoder steps per revolution
    public static final double TPS312 = (312.0/60.0) * 537.7;
    public static ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        // Introducing our devices!! Yayy!!!
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

        // Sets PIDF coefficients for drive system and lift motor using variables
        frontRightMotor.setVelocityPIDFCoefficients(NEW_P_DRIVE,NEW_I_DRIVE,NEW_D_DRIVE,NEW_F_DRIVE);
        frontLeftMotor.setVelocityPIDFCoefficients(NEW_P_DRIVE,NEW_I_DRIVE,NEW_D_DRIVE,NEW_F_DRIVE);
        backRightMotor.setVelocityPIDFCoefficients(NEW_P_DRIVE,NEW_I_DRIVE,NEW_D_DRIVE,NEW_F_DRIVE);
        backLeftMotor.setVelocityPIDFCoefficients(NEW_P_DRIVE,NEW_I_DRIVE,NEW_D_DRIVE,NEW_F_DRIVE);
        liftMotor.setVelocityPIDFCoefficients(NEW_P_LIFT,NEW_I_LIFT,NEW_D_LIFT,NEW_F_LIFT);
        liftMotor.setPositionPIDFCoefficients(NEW_POS_P_LIFT);

        // Make sure lift doesn't fall under gravity
        // Just a failsafe, as setTargetPosition holds at position anyway
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set drive motors to RUN_USING_ENCODER
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reset drive system motor encoders
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Reset runtime variable, not used yet
        runtime.reset();

        // Lift sensorless homing code, will move the lift during initialization
        // Using built-in CurrentAlert is easier
        // liftMotor gets switched back to RUN_TO_POSITION near end of code
        liftMotor.setCurrentAlert(1500, CurrentUnit.MILLIAMPS);
        while (!liftMotor.isOverCurrent()) {
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor.setVelocity(-0.25 * TPS312);
        }
        // This should be setPower to bypass the use of PIDF so it stops instantly
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(0);
        liftMotor.setCurrentAlert(3000, CurrentUnit.MILLIAMPS);

        // Play button is pressed
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Get and assign joystick values. remember, Y stick value is reversed
            // These would normally be y, x, and rx, see next comment
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Calculate angle and magnitude from joystick values
            double driveAngle = Math.atan2(x, y);
            double driveMagnitude = Math.hypot(x, y);

            // Cubic root scaling for driveMagnitude, for improved control at lower speeds
            // The Math.abs is there so that it maintains its sign and doesn't spit out complex numbers
            // Math.cbrt and Math.sqrt are always faster than using Math.pow
            double scaledDriveMagnitude = Math.cbrt(Math.abs(driveMagnitude)) * driveMagnitude;

            // IMU Yaw reset button
            // This button choice was made so that it is hard to hit on accident
            if (gamepad1.back) {
                imu.resetYaw();
            }
            // Set botHeading to robot Yaw from IMU
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            // The evil code for calculating motor powers
            // Desmos used to troubleshoot directions without robot
            // https://www.desmos.com/calculator/ckxjhqekpo
            double frontLeftBackRightMotors = scaledDriveMagnitude * Math.sin(driveAngle + botHeading + 0.25 * Math.PI);
            double frontRightBackLeftMotors = scaledDriveMagnitude * -Math.sin(driveAngle + botHeading - 0.25 * Math.PI);
            double frontLeftPower = frontLeftBackRightMotors + rx;
            double backLeftPower = frontRightBackLeftMotors + rx;
            double frontRightPower = frontRightBackLeftMotors - rx;
            double backRightPower = frontLeftBackRightMotors - rx;

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

            // Lift motor height presets
            // A for bottom, X for middle, Y for top
            if (gamepad1.a) {
                liftMotor.setTargetPosition(0);
            }
            if (gamepad1.x) {
                liftMotor.setTargetPosition(2800);
            }
            if (gamepad1.y) {
                liftMotor.setTargetPosition(4300);
            }
            // Tell liftMotor to run to to target position at 0.5 speed
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(0.75);

            // If liftMotor overcurrents, stop it
            // Would make this zero the encoder if it hits the bottom,
            // But in practice, it could hit something going down and zero itself at the wrong height
            if (liftMotor.isOverCurrent()) {
                liftMotor.setPower(0);
            }
            // Telemetry for lift actual height, lift target height, and lift current draw
            telemetry.addData("Lift Height",liftMotor.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Lift Target",liftMotor.getTargetPosition());
            telemetry.addLine();
            telemetry.addData("Lift Current (Milliamps)",liftMotor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.update();
        }
    }
}