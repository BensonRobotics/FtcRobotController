package org.firstinspires.ftc.teamcode.test;

// Imports and package
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

// Class naming and declaration
@Disabled
@TeleOp(name="Ultrasonic Test and Calibrate", group="Test")
public class Ultrasonic_test extends LinearOpMode {

    private UltrasonicSensor ultrasonic0 = null; // Declare ultrasonic sensor

    @Override
    public void runOpMode() throws InterruptedException { // Initialize OpMode

        // Assign hardware variables
        ultrasonic0 = hardwareMap.get(UltrasonicSensor.class, "ultrasonic0");
        telemetry.speak("Initialized!"); // Initialized!
        telemetry.update();

        waitForStart();
        // OpMode started
        while (opModeIsActive()) { // OpMode loop

            telemetry.speak("Ultrasonic0 Distance: " + ultrasonic0.getUltrasonicLevel());
            telemetry.update();

        } // End while loop
    } // End OpMode void
} // End class Ultrasonic_test