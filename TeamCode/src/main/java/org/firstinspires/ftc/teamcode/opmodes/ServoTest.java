package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Dictionary;
import java.util.Hashtable;

@TeleOp(name="Servo Test", group="Iterative Opmode")
public class ServoTest extends LinearOpMode {
    Servo grabberServo = null;
    Servo grabberPivot = null;
    Servo intakePivot = null;
    CRServo intakeServo = null;

    public void runOpMode() throws InterruptedException {
        grabberServo = hardwareMap.get(Servo .class, "grabberServo");
        grabberPivot = hardwareMap.get(Servo.class, "grabberPivot");
        intakeServo = hardwareMap.get(CRServo .class, "intakeServo");
        intakePivot = hardwareMap.get(Servo.class, "intakePivot");

        Dictionary<Integer, Servo> servos = new Hashtable<>();
        servos.put(0, intakePivot);
        servos.put(1, grabberServo);
        servos.put(2, grabberPivot);
        servos.put(3, intakePivot);

        Integer servoIndex = 1;

        waitForStart();

        grabberPivot.setPosition(0.5524);

        boolean bumperPressed = true;
        while (opModeIsActive()) {
            if (gamepad1.left_bumper && servoIndex < servos.size() && !bumperPressed) {
                servoIndex += 1;
                bumperPressed = true;
            } else if (gamepad1.right_bumper && servoIndex > 0 && !bumperPressed) {
                servoIndex -= 1;
                bumperPressed = true;
            }

            if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
                bumperPressed = false;
            }

            telemetry.addData("Selected Servo", servoIndex);
            telemetry.addData("position", servos.get(servoIndex).getPosition());
            telemetry.update();

            Servo selectedServo = servos.get(servoIndex);

            if (servoIndex == 0) {
                intakeServo.setPower(gamepad1.left_stick_x);
            } else {
                intakeServo.setPower(0);
            }

            if (gamepad1.dpad_up && servoIndex > 0) {
                selectedServo.setPosition(selectedServo.getPosition() + 0.001);
            } else if (gamepad1.dpad_down && servoIndex > 0) {
                selectedServo.setPosition(selectedServo.getPosition() - 0.001);
            }
        }
    }

}
