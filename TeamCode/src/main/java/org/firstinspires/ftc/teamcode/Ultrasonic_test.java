package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
@TeleOp(name="Ultrasonic Test and Calibrate", group="Test")
public class Ultrasonic_test extends LinearOpMode {
    private AnalogInput ultrasonic0 = null;
    private int calibrationStage = 0;
    private boolean aPressed = true;
    private double firstVoltage;
    private double secondVoltage;
    private double calculatedFactor;
    @Override
    public void runOpMode() throws InterruptedException {
        ultrasonic0 = hardwareMap.get(AnalogInput.class, "ultrasonic1");
        telemetry.addData("Status:", "Initialized!");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            if (!gamepad1.a) {
                aPressed = false;
            }
            switch (calibrationStage) {
                case 0:
                    telemetry.speak("1st calibration stage");
                    telemetry.speak("Please position the ultrasonic sensor exactly 0.5 meters away from a wall, and press A.");
                    if (gamepad1.a && !aPressed) {
                        firstVoltage = ultrasonic0.getVoltage();
                        aPressed = true;
                        calibrationStage = 1;
                    }
                    break;
                case 1:
                    telemetry.speak("2nd calibration stage");
                    telemetry.speak("Please position the ultrasonic sensor exactly 1 meter away from a wall, and press A.");
                    if (gamepad1.a && !aPressed) {
                        secondVoltage = ultrasonic0.getVoltage();
                        aPressed = true;
                        calibrationStage = 2;
                    }
                case 2:
                    telemetry.speak("3rd calibration stage");
                    calculatedFactor = ((1 - 0.5) / (secondVoltage - firstVoltage));
                    telemetry.speak("Calibration complete! Factor for converting voltage to distance is " + calculatedFactor);
                    telemetry.speak("Probably won't need this, but factor for converting distance to voltage is " + (1 / (calculatedFactor)));
                    telemetry.speak("Press A to restart calibration if needed.");
                    if (gamepad1.a && !aPressed) {
                        calibrationStage = 0;
                        aPressed = true;
                    }
            }
            telemetry.addData("Ultrasonic0 Voltage:", ultrasonic0.getVoltage());
            telemetry.update();
        }
    }
}