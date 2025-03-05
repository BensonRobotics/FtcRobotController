package org.firstinspires.ftc.teamcode.calibrations;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class ColorDistTest extends LinearOpMode {
    ColorRangeSensor clawColorDist;
    public void runOpMode() {
        clawColorDist = hardwareMap.get(ColorRangeSensor.class, "clawColorDist");
        boolean aDebounce = true; // Debounce for A button press
        boolean sensorLEDEnabled = false; // Indicator of LED state, can't just get state from sensor
        clawColorDist.enableLed(sensorLEDEnabled); // Shut off LED at start to comply with FTC rules

        waitForStart();

        while (opModeIsActive()) { // Loop
            if (!aDebounce) { // If not debouncing
                if (gamepad1.a) { // A is pressed
                    sensorLEDEnabled = !sensorLEDEnabled; // Toggle LED state variable
                    aDebounce = true; // Debounce
                }
            } else { // If debouncing
                if (!gamepad1.a) { // Reset debounce if not pressed
                    aDebounce = false; // Reset debounce
                }
            }
            clawColorDist.enableLed(sensorLEDEnabled); // Set LED state from variable

            // Lots of telemetry
            telemetry.addData("Red: ", clawColorDist.red());
            telemetry.addData("Green: ", clawColorDist.green());
            telemetry.addData("Blue: ", clawColorDist.blue());
            telemetry.addData("Alpha: ", clawColorDist.alpha());
            telemetry.addData("ARGB: ", clawColorDist.argb());
            telemetry.addData("Distance (mm): ", clawColorDist.getDistance(DistanceUnit.MM));
            telemetry.addData("Normalized Colors: ", clawColorDist.getNormalizedColors());
            telemetry.addData("Light Detected: ", clawColorDist.getLightDetected());
            telemetry.addData("Raw Light Detected: ", clawColorDist.getRawLightDetected());
            telemetry.update();
        }
    }
}
