package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
@TeleOp(name="Ultrasonic Test and Calibrate", group="Test")
public class Ultrasonic_test extends LinearOpMode {
    private AnalogInput ultrasonic1 = null;
    @Override
    public void runOpMode() throws InterruptedException {
        ultrasonic1 = hardwareMap.get(AnalogInput.class, "ultrasonic1");
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Ultrasonic1 Voltage", ultrasonic1.getVoltage());
        }
    }
}