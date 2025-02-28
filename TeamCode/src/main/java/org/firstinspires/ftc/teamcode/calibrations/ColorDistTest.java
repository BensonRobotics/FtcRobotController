package org.firstinspires.ftc.teamcode.calibrations;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class ColorDistTest extends LinearOpMode {
    ColorRangeSensor clawColorDist;
    public void runOpMode() {
        while (opModeIsActive()) {
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
