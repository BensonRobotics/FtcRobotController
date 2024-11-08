package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class AutoRotationCalibration extends LinearOpMode {
    // Declare devices
    DcMotorEx frontLeftMotor;
    DcMotorEx frontRightMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx backRightMotor;
    IMU imu;

    public void runOpMode() throws InterruptedException {
        // Declare our devices
        // Make sure your ID's match your configuration
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        imu = hardwareMap.get(IMU.class, "imu");

        // Reverse the left motors to correct for mounting direction
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Zero all motor encoders
        frontLeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Set all motors to run using encoders
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up USB forward
        imu.initialize(parameters);
        imu.resetYaw();

        double previousError = 90;
        double integralPower = 0;

        waitForStart();

        while (opModeIsActive()) { // Slowly rotate the robot until it reaches 90 degrees
            // Get robot heading
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            // Calculate the heading error
            double headingError = 90 + botHeading;
            // Calculate proportional power
            double proportionalPower = (0.5 * headingError) / 90;
            // Calculate integral power
            integralPower += (headingError * 0.1) / 90;
            // Calculate derivative power
            double derivativePower = ((headingError - previousError) * 0.1) / 90;
            // Jot down the current error for next time
            previousError = headingError;

            // Add all the motor powers together to get the final power, run the motors at that
            // Right side is negative to rotate instead of drive
            frontRightMotor.setPower(-0.5 * (proportionalPower + integralPower + derivativePower));
            frontLeftMotor.setPower(0.5 * proportionalPower + integralPower + derivativePower);
            backRightMotor.setPower(-0.5 * (proportionalPower + integralPower + derivativePower));
            backLeftMotor.setPower(0.5 * proportionalPower + integralPower + derivativePower);

            // Gemini in Android Studio is insane
            // Report back the motor positions, heading, and heading error
            telemetry.addData("Right Motors' Average Position", (frontRightMotor.getCurrentPosition() + backRightMotor.getCurrentPosition()) / 2);
            telemetry.addLine();
            telemetry.addData("Left Motors' Average Position", (frontLeftMotor.getCurrentPosition() + backLeftMotor.getCurrentPosition()) / 2);
            telemetry.addLine();
            telemetry.addData("Heading", botHeading);
            telemetry.addLine();
            telemetry.addData("Heading Error", headingError);
            telemetry.update();
        }
    }
}
