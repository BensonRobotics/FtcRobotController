package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Calibrate Sundae Machine Positions", group = "Test")
public class SundaePositionCalibrator extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx conveyorMotor;
    private Servo creamServo;
    private double servoAngle = 0;
    private boolean lastButtonState = true;
    private boolean isServoIdling = false;

    @Override
    public void init() {
        conveyorMotor = hardwareMap.get(DcMotorEx.class, "conveyorMotor");
        creamServo = hardwareMap.get(Servo.class, "creamServo");
        conveyorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        conveyorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        creamServo.setPosition(0);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        if (gamepad1.dpad_up) {
            servoAngle = Math.min(servoAngle + 0.002, 1);
        } else if (gamepad1.dpad_down) {
            servoAngle = Math.max(servoAngle - 0.002, 0);
        }

        if (gamepad1.a) {
            if (!lastButtonState) {
                isServoIdling = !isServoIdling;
            }
        } else {
            lastButtonState = false;
        }

        if (!isServoIdling) {
            creamServo.setPosition(servoAngle);
        } else {
            creamServo.setPosition(0);
        }

        telemetry.addData("Run Time", runtime.toString());
        telemetry.addData("Servo Angle", servoAngle);
        telemetry.addData("Servo Enabled", !isServoIdling);
        telemetry.addData("Conveyor Position", conveyorMotor.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}
