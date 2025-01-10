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

        int sleepTime = 0;
        boolean inZone = false;

        int i = 0;
        while (opModeIsActive()) {
            if (i == 0) {
                velocity.x = 0.6;

                sleepTime = 1000;
            } else if (i > 0 && inZone == false) {
                velocity.y = 0.6;
                velocity.x = 0;

                sleepTime = 0;
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

            sleep(sleepTime);
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
}
