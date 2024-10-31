package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.Objects;

//This is a test from Desmond of the Github, Git, and Android Studio syncing

@TeleOp(name="Basic: Linear OpMode", group="Linear OpMode")
public class TeleOP extends LinearOpMode {
    // Declare OpMode members.

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive = null;

    private Servo grabberServo = null;

    private DcMotorEx liftMotor = null;

    // IMU sensor object
    IMU imu;

    public static final double NEW_P = 1.5;
    public static final double NEW_I = 0.2;
    public static final double NEW_D = 0.1;
    public static final double NEW_F = 12.0;

    public void runOpMode() {
        Initialize();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        double robotAngleToField = 0;

        double liftMotorCurrentThreshold = 3000.0;
        int liftBottomPosition = GetLiftBottomPosition(liftMotorCurrentThreshold);
        liftMotor.setTargetPosition(liftBottomPosition);


        /*Main loop */

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            boolean opModeActive = opModeIsActive();
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

            // Take an input vector from the joysticks and use it to move. Exponential scaling will need to be applied for better control.
//          MoveWithFieldRelativeVector(velocity, robotAngleToField, rotation);
            MoveWithVector(velocity, rotation);

            robotAngleToField = UpdateRobotAngleToField(imu);
        }

    }


    // Main Drive Code

    // Move robot using 4 motor velocity/power values with domains from -1 to 1
    private void MoveRobotWithMotorPowers(double frontLeft, double frontRight, double backLeft, double backRight) {
        // TPS(motorRPM) = (motorRPM / 60) * motorStepsPerRevolution
        // Output is basically the motor's max speed in encoder steps per second, which is what setVelocity uses
        // 537.7 is 312 RPM motor's encoder steps per revolution
        double TPS312 = (312.0 / 60.0) * 537.7;

        frontLeftDrive.setVelocity(frontLeft * TPS312);
        frontRightDrive.setVelocity(frontRight * TPS312);
        backLeftDrive.setVelocity(backLeft * TPS312);
        backRightDrive.setVelocity(backRight * TPS312);
    }


    // Move the robot using a Vector2 representing velocity as an input (relative to robot)
    private void MoveWithVector(Vector2 velocity, double rotation) {
        // This (almost) copy pasted code from Gavin's tutorial video uses trig to figure out the individual motor values that will combine to a desired velocity vector.
        // It rotates the vector by -45°, which results in its components (the legs of the triangle) being oriented at right angles to the robot.
        // Then, it divides each of the components by the max of the two components to gain more speed in certain directions,
        // and multiplies these x and y components by the magnitude of the desired vector.
        // The if statement at the bottom just reduces the magnitude of the vector a little if one of the motor values exceeds the limit of 1

        if (!Objects.equals(velocity, new Vector2(0, 0)) || rotation != 0) {
            double sin = Math.sin(Math.atan2(velocity.y, velocity.x) - Math.PI / 4);
            double cos = Math.cos(Math.atan2(velocity.y, velocity.x) - Math.PI / 4);
            double xComponent = velocity.Magnitude() * cos;
            double yComponent = velocity.Magnitude() * sin;
            double max = Math.max(Math.abs(sin), Math.abs(cos));

            double frontLeftMotorVelocity = xComponent / max + rotation;
            double frontRightMotorVelocity = yComponent / max - rotation;
            double backLeftMotorVelocity = yComponent / max + rotation;
            double backRightMotorVelocity = xComponent / max - rotation;

            if ((velocity.Magnitude() + Math.abs(rotation)) > 1) {
                frontLeftMotorVelocity /= velocity.Magnitude() - rotation;
                frontRightMotorVelocity /= velocity.Magnitude() - rotation;
                backLeftMotorVelocity /= velocity.Magnitude() - rotation;
                backRightMotorVelocity /= velocity.Magnitude() - rotation;
            }

            MoveRobotWithMotorPowers(frontLeftMotorVelocity, frontRightMotorVelocity, backLeftMotorVelocity, backRightMotorVelocity);
        }
    }


    // Field oriented drive

    // Update the robotAngleToField variable using the latest data from the gyro
    public double UpdateRobotAngleToField(IMU imu) {
        // If the driver presses the reset orientation button, reset the Z axis on the IMU
        if (gamepad1.back) {
            telemetry.addData("Yaw", "Resetting\n");
            imu.resetYaw();
        }

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.RADIANS));
        return orientation.getYaw(AngleUnit.RADIANS);
    }

    // Move the robot using a Vector2 representing velocity as an input (relative to robot)
    private void MoveWithFieldRelativeVector(Vector2 velocity, double robotAngleToField, double rotation) {
        Vector2 robotRelativeDirection = velocity.Rotate(robotAngleToField);

        MoveWithVector(robotRelativeDirection, rotation);
    }


    // Servos
    private void UpdateServos() {
//        ArrayList<Double> desiredPositions = new ArrayList<Double>(1);
//        desiredPositions.add(0.0);
//        if (gamepad2.dpad_up & desiredPositions.get(0) <= 1) {
//            desiredPositions.set(0, desiredPositions.get(0) + 0.01);
//        } else if (gamepad2.dpad_down & desiredPositions.get(0) >= 0) {
//            desiredPositions.set(0, desiredPositions.get(0) - 0.01);
//        }
//
//        grabberServo.setPosition(desiredPositions.get(0));
    }


    // Vertical Slide

    // Update the lift motor position using the calibrated bottom position and the current threshold for safety
    private void UpdateLiftMotor(int bottomPosition, double currentThreshold) {
        ArrayList<Integer> liftPositions = new ArrayList<Integer>(3);
        liftPositions.add(bottomPosition);
        liftPositions.add(bottomPosition + 2800);
        liftPositions.add(bottomPosition + 4500);

        if (gamepad2.a || gamepad2.x || gamepad2.y) {
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
            liftMotor.setPower(0.5);

            if (IsOverloaded(liftMotor, currentThreshold)) {
                liftMotor.setPower(0.0);
            }
        }

        if (IsOverloaded(liftMotor, currentThreshold)) {
            liftMotor.setPower(0.0);
        }

        telemetry.addData("liftMotorPosition", liftMotor.getCurrentPosition());
        telemetry.addData("liftMotorCurrent", liftMotor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Lift motor mode", liftMotor.getMode());

//        if (gamepad1.dpad_up) {
//            liftMotor.setPower(0.05);
//        } else if (gamepad1.dpad_down) {
//            liftMotor.setPower(-0.05);
//        } else {
//            liftMotor.setPower(0.0);
//        }
    }

    // Calibrate the lift by slowly running it into the bottom of it's travel, and detecting when the resistance on the motor reaches a certain
    // threshold, indicated by a spike in current draw. Then set the initial position of the lift to the current encoder value, since we know the lift
    // is at the bottom of its travel. This is necessary because of the inevitable drift in the encoders.
    private int GetLiftBottomPosition(double currentThreshold) {
        while (!IsOverloaded(liftMotor, currentThreshold)) {
            liftMotor.setPower(-0.05);
        }
        liftMotor.setPower(0.0);

        telemetry.addData("lift bottom position: ", liftMotor.getCurrentPosition());
        return liftMotor.getCurrentPosition();
    }


    // Input
    private double ScaleStickValue(double stickValue) {
        int speedDenominator = 2;
        if (stickValue < 0) {
            stickValue = -NonlinearInterpolate(0, 1, -stickValue, 1.1);
        } else {
            stickValue = NonlinearInterpolate(0, 1, stickValue, 1.1);
        }
        return stickValue / speedDenominator;
    }


    // Utility:
    private double NonlinearInterpolate(double rangeStart, double rangeEnd, double index, double power) {
        double output = 0;
        // this bit of math interpolates between two numbers with an index using the given exponent. There is a desmos with the equation here:
        // https://www.desmos.com/calculator/st8wrb0oph
        output = index * Math.pow(index / (rangeEnd - rangeStart), power);

        return output;
    }

    private boolean IsOverloaded(DcMotorEx motor, double currentThreshold) {
        return (motor.getCurrent(CurrentUnit.MILLIAMPS) > currentThreshold);
    }


    // Initialization
    private void Initialize() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        grabberServo = hardwareMap.get(Servo.class, "grabberServo");

        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorEx.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorEx.Direction.FORWARD);

        liftMotor.setDirection(DcMotorEx.Direction.REVERSE);


        frontLeftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
    }
}