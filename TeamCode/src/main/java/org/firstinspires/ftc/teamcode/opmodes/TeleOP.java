package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Dictionary;
import java.util.Hashtable;
import java.util.List;
import java.util.Objects;


@TeleOp
public class TeleOP extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive = null;

    private Servo grabberServo = null;

    private Servo grabberPivot = null;

    private CRServo intakeServo = null;

    private Servo intakePivot = null;

    private DcMotorEx liftMotor = null;

    private DcMotorEx horizontalSlideMotor = null;

    // IMU sensor object
    IMU imu;

    public static final double NEW_P = 1.5;
    public static final double NEW_I = 0.2;
    public static final double NEW_D = 0.1;
    public static final double NEW_F = 12.0;

    double liftMotorCurrentThreshold;
    int liftBottomPosition;

    double driveSpeedLimit = 1F;

    /*April Tag Detection*/
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;


    double driveTicksPerSecond = (312.0 * 537.7 / 60.0);

    int currentTransferState = 0;

    boolean transferAutoSequence = false;

    boolean gamepad2xDebouncing = false;

    boolean isLiftHoming = false;

    public void runOpMode() throws InterruptedException{
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");

        horizontalSlideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");

        grabberServo = hardwareMap.get(Servo .class, "grabberServo");
        grabberPivot = hardwareMap.get(Servo.class, "grabberPivot");
        intakeServo = hardwareMap.get(CRServo .class, "intakeServo");
        intakePivot = hardwareMap.get(Servo.class, "intakePivot");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorEx.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorEx.Direction.FORWARD);

        liftMotor.setDirection(DcMotorEx.Direction.REVERSE);

        horizontalSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        horizontalSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double liftMotorCurrentThreshold = 3000.0;

        double slideMotorCurrentThreshold = 2000.0;

        /* Orientation Variables */

        // Define the mounting direction of the control hub
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // define the control hub IMU (Integrated Measurement Unit)
        imu = hardwareMap.get(IMU.class, "imu");
        // Initialize the IMU with this mounting orientation. Note: if two conflicting directions are chosen, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);

        frontRightDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        frontLeftDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        backRightDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        backLeftDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        /*April Tag Detection*/
        InitAprilTag();

        /*Localization*/
        Dictionary<String, Vector3D> currentRobotOrientationData = new Hashtable<>();
        currentRobotOrientationData.put("position", new Vector3D(0, 0, 0));
        currentRobotOrientationData.put("rotation", new Vector3D(0, 0, 0));

        /*Main loop */

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        grabberServo.setPosition(0.7889);
        grabberPivot.setPosition(0.7911);
        intakePivot.setPosition(0.9455);

        telemetry.addData("ready to zero slide", 1);
        updateTelemetry(telemetry);

        int liftBottomPosition = 0;
        liftMotor.setTargetPosition(liftBottomPosition);

        ZeroHorizontalSlideEncoder(slideMotorCurrentThreshold);

        // Run until the end of the match (driver presses STOP)
        boolean grabberPivotHomed = false;
        while (opModeIsActive()) {
            if (runtime.milliseconds() > 500 && !grabberPivotHomed) {
                grabberPivot.setPosition(0.5522);
                grabberPivotHomed = true;
            }

            /*AprilTag stuff*/
            UpdateAprilTagTelemetry();
            currentRobotOrientationData = UpdateRobotOrientationData(imu, currentRobotOrientationData);
            telemetry.addData("Position estimate", currentRobotOrientationData.get("position"));
            telemetry.addData("Yaw estimate", currentRobotOrientationData.get("rotation").getZ());

            // Get needed gamepad joystick values
            double leftStickY = ScaleStickValue(-gamepad1.left_stick_y);  // Note: pushing stick forward gives negative value
            double leftStickX = ScaleStickValue(gamepad1.left_stick_x);
            double rightStickY = ScaleStickValue(-gamepad1.right_stick_y);  // Note: pushing stick forward gives negative value
            double rightStickX = ScaleStickValue(gamepad1.right_stick_x);

            Vector2 velocity = new Vector2(leftStickX, leftStickY);

            double rotation = rightStickX;

            telemetry.addData("velocity:", (velocity.Value()));
            updateTelemetry(telemetry);

            UpdateServos();

            UpdateLiftMotor(liftBottomPosition, liftMotorCurrentThreshold);

            if (!transferAutoSequence) {
                UpdateSlideMotor(slideMotorCurrentThreshold);
            }

            // Take an input vector from the joysticks and use it to move.
            telemetry.addData("robot angle to field", currentRobotOrientationData.get("rotation").getZ());
            MoveWithFieldRelativeVector(velocity, currentRobotOrientationData.get("rotation").getZ(), rotation);

            if (gamepad1.start) {
                ZeroHorizontalSlideEncoder(slideMotorCurrentThreshold);
            } else if (gamepad1.back) {
                isLiftHoming = true;
            }
            if (isLiftHoming) {
                GetLiftBottomPosition(liftMotorCurrentThreshold);
            }

            if (horizontalSlideMotor.getCurrent(CurrentUnit.MILLIAMPS) > slideMotorCurrentThreshold) {
                horizontalSlideMotor.setPower(0);
            }
        }

        visionPortal.close();
    }


    // Main Drive Code

    // Move the robot using a Vector2 representing velocity as an input (relative to robot)
    private void MoveWithFieldRelativeVector(Vector2 velocity, double robotAngleToField, double rotation) {
        Vector2 robotRelativeDirection = velocity.Rotate(-robotAngleToField);
        telemetry.addData("\nrobot relative direction\n", robotRelativeDirection.Value());

        MoveWithVector(robotRelativeDirection, rotation);
    }

    // Move the robot using a Vector2 representing velocity as an input (relative to robot)
    private void MoveWithVector(Vector2 velocity, double rotation) {
        // This (almost) copy pasted code from Gavin's tutorial video uses trig to figure out the individual motor values that will combine to a desired velocity vector.
        // It rotates the vector by -45°, which results in its components (the legs of the triangle) being oriented at right angles to the robot.
        // Then, it divides each of the components by the max of the two components to gain more speed in certain directions,
        // and multiplies these x and y components by the magnitude of the desired vector.
        // The if statement at the bottom just reduces the magnitude of the vector a little if one of the motor values exceeds the limit of 1

        if (!Objects.equals(velocity, new Vector2(0, 0)) || rotation != 0) {
            double driveAngle = Math.atan2(velocity.y, -velocity.x);
            // The evil code for calculating motor powers
            // Desmos used to troubleshoot directions without robot
            // https://www.desmos.com/calculator/3gzff5bzbn
            double frontLeftBackRightMotors = velocity.Magnitude() * Math.sin(driveAngle - 0.25 * Math.PI);
            double frontRightBackLeftMotors = velocity.Magnitude() * Math.cos(driveAngle - 0.25 * Math.PI);
            double frontLeftPower = frontLeftBackRightMotors + Math.min(rotation, 1) * driveSpeedLimit;
            double backLeftPower = frontRightBackLeftMotors + Math.min(rotation, 1) * driveSpeedLimit;
            double frontRightPower = frontRightBackLeftMotors - Math.min(rotation, 1) * driveSpeedLimit;
            double backRightPower = frontLeftBackRightMotors - Math.min(rotation, 1) * driveSpeedLimit;

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
            frontLeftDrive.setVelocity(frontLeftPower * driveTicksPerSecond);
            backLeftDrive.setVelocity(backLeftPower * driveTicksPerSecond);
            frontRightDrive.setVelocity(frontRightPower * driveTicksPerSecond);
            backRightDrive.setVelocity(backRightPower * driveTicksPerSecond);

        }
    }

    // Update all the orientation data using the most recent data from the IMU, AprilTag detection alg, and motor Encoders
    public Dictionary<String, Vector3D> UpdateRobotOrientationData(IMU imu, Dictionary<String, Vector3D> localizationData) {
        // If the driver presses the reset orientation button, reset the Z axis on the IMU
        if (gamepad1.back) {
            telemetry.addData("Yaw", "Resetting\n");
            imu.resetYaw();
            localizationData.put("rotation", new Vector3D(0, 0, 0));

//            telemetry.addData("Resetting position data", "...");
//            localizationData.put("position", new Vector3D(0, 0, 0));
        }

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (currentDetections.size() > 0) {
            localizationData.put("position", GetPositionWithAprilTags(currentDetections));
            //localizationData.put("rotation", new Vector3D(0, 0, GetYawWithAprilTags(currentDetections)));

            if (localizationData.get("rotation").getZ() == 0 && imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) != 0) {
                imu.resetYaw();
            }
        }

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.RADIANS));
        localizationData.put("rotation", new Vector3D(orientation.getPitch(AngleUnit.RADIANS),
                orientation.getRoll(AngleUnit.RADIANS), orientation.getYaw(AngleUnit.RADIANS)));


        return localizationData;
    }

    // Servo
    private void UpdateServos() {
//        grabberServo.setPosition(servo1Power);
        telemetry.addData("Grabber Position", grabberServo.getPosition());
        telemetry.addData("Grabber Pivot", grabberPivot.getPosition());
        telemetry.addData("Intake Pivot", intakePivot.getPosition());

        if (gamepad2.b) {
            if ((horizontalSlideMotor.getCurrentPosition() < 267) && grabberPivot.getPosition() == 0.5522) {
                horizontalSlideMotor.setTargetPosition(267);
                horizontalSlideMotor.setPower(1);
                horizontalSlideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            }
            currentTransferState = 0;
            // Check if slide is too far in, if so, extend it to a safe distance before raising the intake
            intakePivot.setPosition(0.6711); // sub barrier clear / transfer
            grabberPivot.setPosition(0.5522); // transfer standby
            grabberServo.setPosition(0.81); // transfer standby
        } else if (gamepad2.a) {
            if ((horizontalSlideMotor.getCurrentPosition() < 267) && grabberPivot.getPosition() == 0.5522) {
                horizontalSlideMotor.setTargetPosition(267);
                horizontalSlideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                horizontalSlideMotor.setPower(1);
            }
            currentTransferState = 1;
            intakePivot.setPosition(0.9606); // down
            grabberPivot.setPosition(0.5522); // transfer standby
            grabberServo.setPosition(0.81); // transfer standby
        } else if (gamepad2.x && !gamepad2xDebouncing) {
            gamepad2xDebouncing = true;
            transferAutoSequence = true;
            if ((horizontalSlideMotor.getCurrentPosition() < 267) && grabberPivot.getPosition() == 0.5522) {
                horizontalSlideMotor.setTargetPosition(267);
                horizontalSlideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                horizontalSlideMotor.setPower(1);
            }
            currentTransferState = 2;
            // Check if slide is too far in, if so, extend it to a safe distance before raising the intake
            intakePivot.setPosition(0.6711); // sub barrier clear / transfer
            grabberPivot.setPosition(0.5522); // transfer
            grabberServo.setPosition(0.81); // open
            // Bring slide in to transfer position, running intake in reverse to eject extra samples, then close claw (0.9015),
            // bring slide out enough that claw can clear intake, then bring claw up to the depo position, and bring slide back in all the way.
            intakeServo.setPower(-0.75); // Spit out double samples to avoid penalty
            horizontalSlideMotor.setTargetPosition(0);
            horizontalSlideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            horizontalSlideMotor.setPower(1);

        } else {
            gamepad2xDebouncing = false;
            if (gamepad2.y) {
                currentTransferState = 3;
                intakePivot.setPosition(0.9606); // down (It may be better to leave this up until the driver presses A again,
                // but I think maybe the intake should go down in the x sequence as the claw goes up, so that way it's out of the way; I'll just set this as down for now)
                grabberPivot.setPosition(0.8328); // depo
                grabberServo.setPosition(0.6); // open
            }
        }

        intakeServo.setPower(ScaleStickValue(-gamepad2.right_stick_y));

        if (transferAutoSequence && !horizontalSlideMotor.isBusy()) {
            grabberServo.setPosition(0.9222);
            double grabSampleTimeOld = runtime.milliseconds();
            if (runtime.milliseconds() - grabSampleTimeOld > 500) { // Sample has been grabbed
                horizontalSlideMotor.setTargetPosition(267);
                horizontalSlideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                horizontalSlideMotor.setPower(1);
                intakePivot.setPosition(0.9606); // down
                transferAutoSequence = false;
            }
        }

        if (!horizontalSlideMotor.isBusy()) {
            horizontalSlideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        // Servo position reference:
        //  Intake pivot up (transfer and barrier clear) : 0.6711
        //  Intake pivot down (ground level) : 0.9606
        //  Grabber pivot initial position : 0.7911
        //  Grabber pivot standby / transfer position : 0.5522
        //  Grabber pivot up / depo : currently unknown, probably somewhere close to 0.8?
        //  Grabber open / ready for transfer : 0.81
        //  Grabber closed : 0.9015 (this might be a bit tight, but it doesn't hurt)
        //  Grabber wide open / depo : 0.6 (the regular open / transfer position would probably work just fine here but I guess it doesn't hurt to open it extra wide here)
        //  Remember, to set up place the grabber arm over the raised intake and open the claw all the way


        // Grabber pivot code.
        // position 0 is down to floor, position 1 is 90 degrees up to sample transfer
        // Also, horizontal slide cannot retract fully if grabberPivot is below 90 degrees
        // I should make the horizontal slide retract button also raise grabberPivot to 90 degrees
//        if (gamepad2.right_stick_x > 0.8) {
//            intakePivot.setPosition(0);
//        } else if (gamepad2.right_stick_x < -0.8) {
//            intakePivot.setPosition(1);
//        }
//
//
//        if (gamepad2.left_stick_x > 0.8) {
//            grabberServo.setPosition(0.87);
//        } else if (gamepad2.left_stick_x < -0.8) {
//            grabberServo.setPosition(0.775);
//        }
//
//        if (gamepad2.left_stick_y > 0.8) {
//            grabberPivot.setPosition(0.6761);
//        } else if (gamepad2.left_stick_y < -0.8) {
//            grabberServo.setPosition(0.4639);
//        }
    }

    // horizontal Slide
    private void UpdateSlideMotor(double slideMotorCurrentThreshold) {
        double horizontalSlideVelocity = 0;

        horizontalSlideVelocity = gamepad2.right_trigger - gamepad2.left_trigger;

        // Encoder based limits
        if ((horizontalSlideMotor.getCurrentPosition() <= 0 &&
                horizontalSlideVelocity < 0) ||
                (horizontalSlideMotor.getCurrentPosition() >= 2450 &&
                        horizontalSlideVelocity > 0)) {
            horizontalSlideVelocity = 0;
            telemetry.addData("slideStopped", 1);
        }

        telemetry.addData("slidePosition", horizontalSlideMotor.getCurrentPosition());
        telemetry.addData("targetSlideVelocity", horizontalSlideVelocity);


        horizontalSlideMotor.setPower(horizontalSlideVelocity);
    }

    private void ZeroHorizontalSlideEncoder(double slideMotorCurrentThreshold) throws InterruptedException {
        telemetry.addData("zeroing             slide", 1);
        updateTelemetry(telemetry);
        horizontalSlideMotor.setPower(0);
//        grabberPivot.setPosition(0.5);
//        while (grabberPivot.getPosition() != 0.5) {
//            telemetry.addData("Homing grabber pivot", 1);
//        }

        horizontalSlideMotor.setPower(-1);
        while (!IsOverloaded(horizontalSlideMotor, slideMotorCurrentThreshold)) {
            telemetry.addData("zeroing slide", 1);
            updateTelemetry(telemetry);
        }
        horizontalSlideMotor.setPower(0);

        telemetry.addData("slide in position ", horizontalSlideMotor.getCurrentPosition());
        horizontalSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Vertical Slide

    // Update the lift motor position using the calibrated bottom position and the current threshold for safety
    private void UpdateLiftMotor(int bottomPosition, double currentThreshold) {
        ArrayList<Integer> liftPositions = new ArrayList<Integer>(3);
        liftPositions.add(bottomPosition);
        liftPositions.add(bottomPosition + 2000);
        liftPositions.add(bottomPosition + 4200);

        if (gamepad1.a || gamepad1.x || gamepad1.y) {
            // Since the variable is initialized to 0, and if we are running this code we know either a, x, or y has been pressed,
            // we can skip the conditional for one of the buttons, in this case a.
            int desiredPositionIndex = 0;
            if (gamepad2.x) {
                desiredPositionIndex = 1;
            } else if (gamepad2.y) {
                desiredPositionIndex = 2;
            }

            int desiredPosition = liftPositions.get(desiredPositionIndex);

            liftMotor.setTargetPosition(desiredPosition);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(1.0);

            if (IsOverloaded(liftMotor, currentThreshold)) {
                liftMotor.setPower(0.0);
            }
        }

        if (!IsOverloaded(liftMotor, currentThreshold)) {
            liftMotor.setPower(1.0);
        } else {
            liftMotor.setPower(0.0);
        }

        telemetry.addData("liftMotorPosition", liftMotor.getCurrentPosition());
        telemetry.addData("liftMotorCurrent", liftMotor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Lift motor mode", liftMotor.getMode());

//        double rightStickY = -gamepad2.right_stick_y;
//        if (rightStickY != 0) {
//            liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//            liftMotor.setPower(rightStickY);
//        } else {
//            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
    }

    // Calibrate the lift by slowly running it into the bottom of it's travel, and detecting when the resistance on the motor reaches a certain
    // threshold, indicated by a spike in current draw. Then set the initial position of the lift to the current encoder value, since we know the lift
    // is at the bottom of its travel. This is necessary because of the inevitable drift in the encoders.
    private void GetLiftBottomPosition(double currentThreshold) {
        if (!IsOverloaded(liftMotor, currentThreshold)) {
            liftMotor.setPower(-0.5);
            telemetry.addData("zeroing lift", IsOverloaded(liftMotor, currentThreshold));
            updateTelemetry(telemetry);
        } else {
            liftMotor.setPower(0.0);
            telemetry.addData("lift bottom position: ", liftMotor.getCurrentPosition());
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            isLiftHoming = false;
        }
    }


    // Input
    private double ScaleStickValue(double stickValue) {
        int speedDenominator = 1;
        if (stickValue < 0) {
            stickValue = -NonlinearInterpolate(0, 1, -stickValue, 1.2);
        } else {
            stickValue = NonlinearInterpolate(0, 1, stickValue, 1.2);
        }
        return stickValue / speedDenominator;
    }


    // Utility:
    private double NonlinearInterpolate(double rangeStart, double rangeEnd, double index, double power) {
        // this bit of math interpolates between two numbers with an index using the given exponent. There is a desmos with the equation here:
        // https://www.desmos.com/calculator/st8wrb0oph
        double output = index * Math.pow(index / (rangeEnd - rangeStart), power);

        return output;
    }

    private boolean IsOverloaded(DcMotorEx motor, double currentThreshold) {
        return (motor.getCurrent(CurrentUnit.MILLIAMPS) > currentThreshold);
    }

    private void InitAprilTag() {
        Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0, 0);

        // Create the AprilTag processor - all these values are blind guesses as none of it has been tested yet
        aprilTag = new AprilTagProcessor.Builder().
                setDrawTagOutline(true)
                .setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary())
                .setOutputUnits(DistanceUnit.MM, AngleUnit.RADIANS)
                .build();

        // Again just a guess for what value we'll want. According to the example program this should be able to detect a 2" tag from 6 feet away at 22 FPS
        aprilTag.setDecimation(2);

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Set camera resolution - another guess
        builder.setCameraResolution(new Size(640, 480));

        builder.enableLiveView(true);

        // Set stream format; apparently the MJPEG format uses less bandwidth than YUY2
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        builder.setAutoStopLiveView(true);

        // Set and enable the processor
        builder.addProcessor(aprilTag);

        visionPortal = builder.build();

        // Enable or disable the aprilTag processor
        visionPortal.setProcessorEnabled(aprilTag, true);
    }

    private void UpdateAprilTagTelemetry() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Iterate through the list of detections and display the info for each one
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (mm)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (rad)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (mm, rad, rad)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
    }

    private Vector3D GetPositionWithAprilTags(List<AprilTagDetection> detections) {
//        List<Vector3D> positionEstimates = Collections.emptyList();
//        telemetry.addData("detections[0]", detections.get(0));
//
//        for (AprilTagDetection detection : detections) {
//            if (detection.metadata != null) {
//                positionEstimates.add(new Vector3D(detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//            }
//            telemetry.addData("!detections[0]", detection);
//        }
//
//        return Vector3DAverage(positionEstimates);

        return new Vector3D(0, 0, 0);
    }

    private double GetYawWithAprilTags(List<AprilTagDetection> detections) {
//        double yawEstimateSum = 0.0;
//
//        for (AprilTagDetection detection : detections) {
//            if (detection.metadata != null) {
//                yawEstimateSum += detection.ftcPose.yaw;
//            }
//        }
//
//        return (yawEstimateSum / detections.size());

        return 0.0;
    }

    private Vector3D Vector3DAverage(List<Vector3D> vectors) {
        double xSum = 0;
        double ySum = 0;
        double zSum = 0;

        for (Vector3D vector : vectors) {
            xSum += vector.getX();
            ySum += vector.getY();
            zSum += vector.getZ();
        }

        return new Vector3D((xSum / vectors.size()), (ySum / vectors.size()), (zSum / vectors.size()));
    }
}
