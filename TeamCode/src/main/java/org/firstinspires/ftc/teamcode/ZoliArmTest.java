package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Zoli Arm Test", group = "Practice" )
public class ZoliArmTest extends OpMode {

    private Servo wristServo = null;

    @Override
    public void init() {
        wristServo = hardwareMap.get(Servo.class, "wrist");
        wristServo.setPosition(0.4);
    }
    @Override
    public void start() {
    wristServo.setPosition(0.8);
    }
    @Override
    public void loop() {
        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    wristServo.setPosition(0.1);
        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        wristServo.setPosition(0.6);
    }

}
