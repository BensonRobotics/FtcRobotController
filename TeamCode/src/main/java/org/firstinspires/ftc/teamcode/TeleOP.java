package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp(name="Basic: Linear OpMode", group="Linear OpMode")
public class TeleOP extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive = null;

    // IMU sensor object
    IMU imu;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "left_drive");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "right_drive");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "left_drive");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "right_drive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorEx.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorEx.Direction.FORWARD);

        // Get needed gamepad joystick values
        double leftStickY = scaleStickValue(-gamepad1.left_stick_y);  // Note: pushing stick forward gives negative value
        double leftStickX =  scaleStickValue(gamepad1.left_stick_x);
        double rightStickY =  scaleStickValue(-gamepad1.right_stick_y);  // Note: pushing stick forward gives negative value
        double rightStickX =  scaleStickValue(gamepad1.right_stick_x);

        Vector2 velocity = new Vector2(leftStickX, leftStickY);

        /* Orientation Variables */
        double robotAngleToField = 0;

        // Define the mounting direction of the control hub
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // define the control hub IMU (Integrated Measurement Unit)
        imu = hardwareMap.get(IMU.class, "imu");
        // Initialize the IMU with this mounting orientation. Note: if two conflicting directions are chosen, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        /*Main loop */

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Take an input vector from the joysticks and use it to move. Exponential scaling will need to be applied for better control.
            moveWithFieldRelativeVector(velocity, robotAngleToField, rightStickX);

            robotAngleToField = updateRobotAngleToField(imu);
        }

    }

    private void moveRobotWithMotorPowers(double frontLeft, double frontRight, double backLeft, double backRight) {
        frontLeftDrive.setVelocity(frontLeft);
        frontRightDrive.setVelocity(frontRight);
        backLeftDrive.setVelocity(backLeft);
        backRightDrive.setVelocity(backRight);
    }

    // Update the robotAngleToField variable using the latest data from the gyro
    public double updateRobotAngleToField(IMU imu) {
        // If the driver presses the reset orientation button, reset the Z axis on the IMU
        if (gamepad1.y) {
            telemetry.addData("Yaw", "Resetting\n");
            imu.resetYaw();
        }

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    // Move the robot using a Vector2 representing velocity as an input (relative to robot)
    private void moveWithFieldRelativeVector(Vector2 velocity, double robotAngleToField, double rotation) {
        Vector2 robotRelativeDirection = velocity.Rotate(robotAngleToField);

        MoveWithVector(robotRelativeDirection, rotation);
    }

    // Move the robot using a Vector2 representing velocity as an input (relative to robot)
    private void MoveWithVector(Vector2 velocity, double rotation) {
        // This copy pasted code from Gavin's tutorial video uses trig to figure out the individual motor values that will combine to a desired velocity vector.
        // It rotates the vector by -45°, which results in its components (the legs of the triangle) being oriented at right angles to the robot.
        // Then, it divides each of the components by the max of the two components to gain more speed in certain directions,
        // and multiplies these x and y components by the magnitude of the desired vector.
        // The if statement at the bottom just reduces the magnitude of the vector a little if one of the motor values exceeds the limit of 1

        double xComponent = Math.cos(Math.atan2(velocity.y, velocity.x) - Math.PI / 4);
        double yComponent = Math.sin(Math.atan2(velocity.y, velocity.x) - Math.PI / 4);
        double max = Math.max(Math.abs(xComponent), Math.abs(yComponent));

        double xMotorVelocity = xComponent/max * velocity.Magnitude() + rotation;
        double yMotorVelocity = yComponent/max * velocity.Magnitude() - rotation;

        if ((velocity.Magnitude() + Math.abs(rotation) > 1)) {
            xMotorVelocity /= velocity.Magnitude() + rotation;
            yMotorVelocity /= velocity.Magnitude() + rotation;
        }

        moveRobotWithMotorPowers(xMotorVelocity, yMotorVelocity, yMotorVelocity, xMotorVelocity);
    }

    private double scaleStickValue(double stickValue) {
        if (stickValue < 0) {
            stickValue = -nonlinearInterpolate(0, 1, -stickValue, "quadratic");
        } else {
            stickValue = nonlinearInterpolate(0, 1, stickValue, "quadratic");
        }
        return stickValue;
    }

    private double nonlinearInterpolate(double rangeStart, double rangeEnd, double index, String scalingType) {
        double output = 0;
        switch (scalingType) {
            case "quadratic":
                // quadratic
                // this bit of math interpolates between two numbers with an index using the given exponent. There is a desmos with the equation here:
                // https://www.desmos.com/calculator/st8wrb0oph
                output = index * Math.pow(index * ((index - rangeStart)/(rangeEnd - rangeStart)), 2);
                break;
            case "cubic":
                // cubic
                output = index * Math.pow(index * ((index - rangeStart)/(rangeEnd - rangeStart)), 3);
                break;
            }
            return output;
        }
    }