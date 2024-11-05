package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class Auto extends LinearOpMode {

    public static final double NEW_POS_P_DRIVE = 1.0;

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

        frontLeftMotor.setPositionPIDFCoefficients(NEW_POS_P_DRIVE);
        frontRightMotor.setPositionPIDFCoefficients(NEW_POS_P_DRIVE);
        backLeftMotor.setPositionPIDFCoefficients(NEW_POS_P_DRIVE);
        backRightMotor.setPositionPIDFCoefficients(NEW_POS_P_DRIVE);

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Calculations to convert linear distance in mm to encoder steps
        double stepsPerMMDrive = 537.7/104.0*Math.PI;
        // Set targetSteps to distance in MM, converted to encoder steps
        // (int) casts float value to int for rounding down
        int targetSteps = (int) (stepsPerMMDrive*1000);

        // Zero all motor encoders
        frontLeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        // Set all motor target positions
        frontLeftMotor.setTargetPosition(targetSteps);
        backLeftMotor.setTargetPosition(targetSteps);
        frontRightMotor.setTargetPosition(targetSteps);
        backRightMotor.setTargetPosition(targetSteps);

        // Set all motor power levels
        frontRightMotor.setPower(0.5);
        frontLeftMotor.setPower(0.5);
        backRightMotor.setPower(0.5);
        backLeftMotor.setPower(0.5);

        // Tell all motors to run to target position
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
}
