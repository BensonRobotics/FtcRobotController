package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Autonomous(name="Auto1",group="187auto ")
public class Auto extends LinearOpMode {
    public static final double NEW_P = 1.5;
    public static final double NEW_I = 0.2;
    public static final double NEW_D = 0.1;
    public static final double NEW_F = 12.0;

    public void runOpMode(){
        DcMotorEx frontLeftMotor;
        DcMotorEx frontRightMotor;
        DcMotorEx backLeftMotor;
        DcMotorEx backRightMotor;
        // Declare our motors
        // Make sure your ID's match your configuration
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        double stepsPerMMDrive = 537.7/104.0;
        int stepsTraveled = (int) (stepsPerMMDrive*200);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);

        frontRightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        frontLeftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        backRightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        backLeftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        waitForStart();

        // Encoder on left motors may be reversed by setDirection, may not.
        // If so, add - sign to left motors' mmTraveled
        frontLeftMotor.setTargetPosition(stepsTraveled);
        backLeftMotor.setTargetPosition(stepsTraveled);
        frontRightMotor.setTargetPosition(stepsTraveled);
        backRightMotor.setTargetPosition(stepsTraveled);

    }
}
