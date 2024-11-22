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

    // Same thing but for the linear slide motor
    float slideTicksPerSecond = (float) (30.0 * 5281.1 / 60.0);

    // liftStepsPerMM = liftMotorStepsPerRevolution / (liftPulleyPitchDiameter * PI)
    // Output is how many encoder steps per mm of lift height
    double liftStepsPerMM = 537.7 / (38.2 * Math.PI);
    boolean isLiftHoming = false;
    ElapsedTime runtime = new ElapsedTime();
    boolean useFieldCentric = true;
    boolean useLift = true;
    boolean useDiscreteLift = true;
    boolean useDiscreteSlide = false;
    boolean isSlideRestricted;
    short liftCurrentAlert = 2500;
    float driveSpeedLimit = 0.75F;
    float liftSpeedLimit = 0.75F;

    @Override
    public void runOpMode() throws InterruptedException {

        // Declare our devices!! Yayy!!!
        DcMotorEx frontLeftMotor;
        DcMotorEx frontRightMotor;
        DcMotorEx backLeftMotor;
        DcMotorEx backRightMotor;
        DcMotorEx liftMotor;
        DcMotorEx slideMotor;
        CRServo grabberServo;

        // Assign our devices
        // Make sure your ID's match your configuration
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        grabberServo = hardwareMap.get(CRServo.class, "grabberServo");

        // Apply motor PIDF coefficients
        frontLeftMotor.setVelocityPIDFCoefficients(NEW_P_DRIVE,NEW_I_DRIVE,NEW_D_DRIVE,NEW_F_DRIVE);
        frontRightMotor.setVelocityPIDFCoefficients(NEW_P_DRIVE,NEW_I_DRIVE,NEW_D_DRIVE,NEW_F_DRIVE);
        backLeftMotor.setVelocityPIDFCoefficients(NEW_P_DRIVE,NEW_I_DRIVE,NEW_D_DRIVE,NEW_F_DRIVE);
        backRightMotor.setVelocityPIDFCoefficients(NEW_P_DRIVE,NEW_I_DRIVE,NEW_D_DRIVE,NEW_F_DRIVE);

        // Reverse motors that are backwards otherwise
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
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset drive system motor encoders
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set drive motors to RUN_USING_ENCODER
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setTargetPosition(0);
        // Default is 5 ticks
        liftMotor.setTargetPositionTolerance(10);

        // Make sure motors don't run from the get-go
        grabberServo.setPower(0);

        // Set lift motor current trip
        liftMotor.setCurrentAlert(liftCurrentAlert, CurrentUnit.MILLIAMPS);

        // Reset runtime variable, not used yet
        runtime.reset();

        double botHeading = 0;

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

            /*
            Here's something you can use for acceleration or scaling or something
            it's the hyperbolic tangent function, and it makes a nice s-curve
            centered around 0,0. It is definitely more compact and faster than a piecewise quadratic
            Here's the link if you want to mess around with it:
            https://www.desmos.com/calculator/ixxmpzbbya
             */

            // Calculate angle and magnitude from joystick values
            double driveAngle = Math.atan2(x, y);
            double driveMagnitude = Math.hypot(x, y);

            double turnAngle = Math.atan2(rx, ry);
            double turnMagnitude = Math.hypot(rx, ry);

            // Better way of setting speed limit
            driveMagnitude = driveMagnitude * driveSpeedLimit;
            rx = rx * driveSpeedLimit;
            // IMU Yaw reset button
            // This button choice was made so that it is hard to hit on accident
            if (gamepad1.back) {
                imu.resetYaw();
            }
            // Set botHeading to robot Yaw from IMU, if used
            if (useFieldCentric) {
                botHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            }
            // The evil code for calculating motor powers
            // Desmos used to troubleshoot directions without robot
            // https://www.desmos.com/calculator/3gzff5bzbn
            double frontLeftBackRightMotors = driveMagnitude * Math.sin(driveAngle - botHeading + 0.25 * Math.PI);
            double frontRightBackLeftMotors = driveMagnitude * -Math.sin(driveAngle - botHeading - 0.25 * Math.PI);
            double frontLeftPower = frontLeftBackRightMotors + rx;
            double backLeftPower = frontRightBackLeftMotors + rx;
            double frontRightPower = frontRightBackLeftMotors - rx;
            double backRightPower = frontLeftBackRightMotors - rx;

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

            if (useLift) {
                if (useDiscreteLift) { // If using discrete lift
                    // Lift homing button
                    if (gamepad1.start) {
                        isLiftHoming = true;
                    }
                    // Lift motor height presets in MM from bottom position
                    // A for bottom, X for middle, Y for top
                    // Please comment what each height preset means
                    if (!isLiftHoming) {
                        if (gamepad1.a) {
                            // Bottom position
                            liftMotor.setTargetPosition(5);
                        }
                        if (gamepad1.x) {
                            //
                            liftMotor.setTargetPosition((int) (300 * liftStepsPerMM));
                        }
                        if (gamepad1.y) {
                            //
                            liftMotor.setTargetPosition((int) (670 * liftStepsPerMM));
                        }
                        // Tell liftMotor to run to to target position at set speed
                        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftMotor.setPower(liftSpeedLimit);

                    } else { // If liftMotor is in fact homing
                        // Lift homing code
                        liftMotor.setCurrentAlert(1500, CurrentUnit.MILLIAMPS);
                        if (!liftMotor.isOverCurrent()) {
                            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            liftMotor.setPower(-0.25);
                        } else { // Once homing is finished
                            liftMotor.setPower(0);
                            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            liftMotor.setTargetPosition(0);
                            liftMotor.setCurrentAlert(liftCurrentAlert, CurrentUnit.MILLIAMPS);
                            isLiftHoming = false;
                        }
                    }
                } else { // If not using discrete lift
                    // Lift is controlled by right stick Y axis
                    // Engages RUN_TO_POSITION when slide stops moving for active position holding
                    if (ry != 0) {
                        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        liftMotor.setPower(liftSpeedLimit * ry);
                    } else if (liftMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                        liftMotor.setTargetPosition(liftMotor.getCurrentPosition());
                        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftMotor.setPower(liftSpeedLimit);
                    }
                    // Lift motor current trip, only if going up, to allow for potential hanging
                    // Remember to tighten the belt on the viper slide to prevent skipping
                    if (liftMotor.isOverCurrent() && ry > 0) {
                        liftMotor.setPower(0);
                    }
                }
            }

            // Linear slide code
            if (!useDiscreteSlide) { // If using joystick slide control
                if (!(((slideMotor.getCurrentPosition() <= 0) && ry <= 0) ||
                        ((slideMotor.getCurrentPosition() >= 2500) && ry >= 0))) { // If lift isn't running into a limit
                    slideMotor.setVelocity(slideTicksPerSecond * ry);
                    isSlideRestricted = false;
                } else { // If lift is running into a limit
                    slideMotor.setVelocity(0);
                    isSlideRestricted = true;
                }
            } else { // If using discrete slide control
                // I will implement inverse kinematics, I swear

            }

            // Grabber servo code. Super complicated.
            // If you let go of right bumper, servo will stop
            // If you let go of left bumper, servo will stop
            // Code arranged in latch formation in case you want either direction to latch
            if (gamepad1.right_bumper) {
                grabberServo.setPower(1);
            } else if (grabberServo.getPower()>0){
                grabberServo.setPower(0);
            }
            if (gamepad1.left_bumper) {
                grabberServo.setPower(-1);
            } else if (grabberServo.getPower()<0) {
                grabberServo.setPower(0);
            }

            // Telemetry
            telemetry.addData("Lift Height:",liftMotor.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Lift Target:",liftMotor.getTargetPosition());
            telemetry.addLine();
            telemetry.addData("Lift Current (Milliamps):",liftMotor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addLine();
            telemetry.addData("Lift Homing:",isLiftHoming);
            telemetry.addLine();
            telemetry.addData("Slide Limited:", isSlideRestricted);
            telemetry.addLine();
            telemetry.addData("Slide Position", slideMotor.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Slide Current:", slideMotor.getCurrent(CurrentUnit.MILLIAMPS));

            telemetry.update();
        }
    }
}