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

        waitForStart();

        while (opModeIsActive()) {
            // Slowly rotate the robot until it reaches 90 degrees
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double headingError = 90 + botHeading;
            double proportionalPower = 0.25 * headingError / 90;
            double integralPower = 0;
            double previousError = 0;
            double derivativePower = 0;
            integralPower += headingError * 0.1;
            frontRightMotor.setPower(-(proportionalPower + integralPower));
            frontLeftMotor.setPower(proportionalPower + integralPower);
            backRightMotor.setPower(-(proportionalPower + integralPower));
            backLeftMotor.setPower(proportionalPower + integralPower);

            // Gemini in Android Studio is insane
            // Report back the motor positions, heading, and heading error
            telemetry.addData("Right Motors", frontRightMotor.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Left Motors", frontLeftMotor.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Heading", botHeading);
            telemetry.addLine();
            telemetry.addData("Heading Error", headingError);
            telemetry.update();
        }
    }
}
