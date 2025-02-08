package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Objects;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Dictionary;
import java.util.Hashtable;
import java.util.Objects;

@Autonomous(name="NoahColorSensorAuto", group="Robot")
public class NoahColorSensorAuto extends LinearOpMode {
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive = null;
    private ElapsedTime runtime = new ElapsedTime();

    double driveTicksPerSecond = (312.0 * 537.7 / 60.0);


    private NormalizedColorSensor colorSensor = null;

    public void runOpMode() throws InterruptedException {
        frontLeftDrive = hardwareMap.get(DcMotorEx .class, "frontLeftMotor");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        Vector2 velocity = new Vector2(0, 0);
        double rotation = 0;


        waitForStart();

        boolean inZone = false;

        while (opModeIsActive()) {
            telemetry.addData("backleftdrive", backLeftDrive.getCurrentPosition());
            telemetry.addData("in zone", inZone);
            telemetry.update();

            if (backLeftDrive.getCurrentPosition() < 350) {
                velocity.x = 0.3;
                velocity.y = 0;
            } else if (runtime.milliseconds() < 3000 && inZone == false) {
                velocity.x = 0;
                velocity.y = 0.1;
            }

            if (inZone) {
                velocity.x = 0;
                velocity.y = 0;

                DepositSample();
            }

            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            if (colors.red > 0.8 || colors.blue > 0.8) {
                inZone = true;
            }

            MoveWithVector(velocity, rotation);
        }
    }

    private void DepositSample() {
        // raise lift
        // open claw
    }

    // Move robot using 4 motor velocity/power values with domains from -1 to 1
    private void MoveRobotWithMotorPowers(double frontLeft, double frontRight, double backLeft, double backRight) {
        // TPS(motorRPM) = (motorRPM / 60) * motorStepsPerRevolution
        // Output is basically the motor's max speed in encoder steps per second, which is what setVelocity uses
        // 537.7 is 312 RPM motor's encoder steps per revolution
        double TPS312 = (312.0 / 60.0) * 537.7 * 0.65;

        frontLeftDrive.setVelocity(frontLeft * TPS312);
        frontRightDrive.setVelocity(frontRight * TPS312);
        backLeftDrive.setVelocity(backLeft * TPS312);
        backRightDrive.setVelocity(backRight * TPS312);
    }

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
            double frontLeftPower = frontLeftBackRightMotors + Math.min(rotation, 1);
            double backLeftPower = frontRightBackLeftMotors + Math.min(rotation, 1);
            double frontRightPower = frontRightBackLeftMotors - Math.min(rotation, 1);
            double backRightPower = frontLeftBackRightMotors - Math.min(rotation, 1);

            // The Great Cleaving approaches
            // Forgive me for what I'm about to do, I took a melatonin an hour ago and I want to collapse onto my bed at this point
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio
            double denominator = Math.max(Math.max(Math.max(Math.abs(frontRightPower), Math.abs(frontLeftPower)),
                    Math.max(Math.abs(backRightPower), Math.abs(backLeftPower))), 1);

            // CLEAVING TIME
            frontRightPower /= denominator;
            frontLeftPower /= denominator;
            backRightPower /= denominator;
            backLeftPower /= denominator;

            telemetry.addData("FL", frontLeftDrive.getVelocity());
            telemetry.addData("FR", frontRightDrive.getVelocity());
            telemetry.addData("BL", backLeftDrive.getVelocity());
            telemetry.addData("BR", backRightDrive.getVelocity());

            // Set motor velocities, converted from (-1 to 1) to (-driveTicksPerSecond to driveTicksPerSecond)
            frontLeftDrive.setVelocity(frontLeftPower * driveTicksPerSecond);
            backLeftDrive.setVelocity(backLeftPower * driveTicksPerSecond);
            frontRightDrive.setVelocity(frontRightPower * driveTicksPerSecond);
            backRightDrive.setVelocity(backRightPower * driveTicksPerSecond);
        }
    }
}
