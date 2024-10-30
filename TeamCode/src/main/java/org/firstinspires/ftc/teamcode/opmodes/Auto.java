package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Auto1",group="187auto ")
public class Auto extends LinearOpMode {
    // PIDF stands for Proportional, Integral, Derivative, Feedforward
    // PIDF coefficients for drive system's setPosition
    public static final double NEW_P_DRIVE = 0.5;
    public static final double NEW_I_DRIVE = 0.1;
    public static final double NEW_D_DRIVE = 0.1;
    public static final double NEW_F_DRIVE = 10.0;

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
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Calculations to convert linear distance in mm to encoder steps
        double stepsPerMMDrive = 537.7/104.0*Math.PI;
        // Set stepsTraveled to distance in mm converted to encoder steps
        // (int) casts float value to int for rounding down
        int stepsTraveled = (int) (stepsPerMMDrive*200);

        // Zero all motor encoders
        frontLeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setTargetPosition(0);
        frontLeftMotor.setTargetPosition(0);
        backRightMotor.setTargetPosition(0);
        backLeftMotor.setTargetPosition(0);

        // Set drive motors' Position Proportional Coefficients
        frontRightMotor.setPositionPIDFCoefficients(NEW_P_DRIVE);
        frontLeftMotor.setPositionPIDFCoefficients(NEW_P_DRIVE);
        backRightMotor.setPositionPIDFCoefficients(NEW_P_DRIVE);
        backLeftMotor.setPositionPIDFCoefficients(NEW_P_DRIVE);

        waitForStart();

        // Encoder on left motors may be reversed by setDirection, may not.
        // If so, add - sign to left motors' stepsTraveled
        frontLeftMotor.setTargetPosition(stepsTraveled);
        backLeftMotor.setTargetPosition(stepsTraveled);
        frontRightMotor.setTargetPosition(stepsTraveled);
        backRightMotor.setTargetPosition(stepsTraveled);

        // Tell all motors to run to target position
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

    }
}
