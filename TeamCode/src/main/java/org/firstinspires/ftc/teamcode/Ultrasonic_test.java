package org.firstinspires.ftc.teamcode;
// Imports and package
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

// Class naming and declaration
@TeleOp(name="Ultrasonic Test and Calibrate", group="Test")
public class Ultrasonic_test extends LinearOpMode {
    private AnalogInput ultrasonic0 = null; // Declare ultrasonic sensor
    private int calibrationStage = 0; // Set calibration stage to 0
    private boolean aPressed = true; // Debounce for A button, start as true in case the A button is pressed
    private double firstVoltage; // First voltage reading at distance
    private double secondVoltage; // Second voltage reading at distance
    private double calculatedFactor; // Calculated factor for converting voltage to distance

    @Override
    public void runOpMode() throws InterruptedException { // Initialize OpMode

        // Assign hardware variables
        ultrasonic0 = hardwareMap.get(AnalogInput.class, "ultrasonic1");
        telemetry.addData("Status:", "Initialized!"); // Initialized!
        telemetry.update();

        waitForStart();
        // OpMode started
        while (opModeIsActive()) { // OpMode loop

            if (!gamepad1.a) { // Reset debounce for A button when not pressed
                aPressed = false;
            }

            switch (calibrationStage) { // Calibration sequence based on calibration stage
                case 0: // Calibration stage 0
                    telemetry.speak("Please position the ultrasonic sensor exactly 0.5 meters away from a wall, and press A.");

                    if (gamepad1.a && !aPressed) { // If A button is pressed and debounced
                        firstVoltage = ultrasonic0.getVoltage(); // Read first voltage and store value
                        aPressed = true; // Engage debouncing
                        calibrationStage = 1; // Advance to calibration stage 1
                    } // End if

                    break; // Don't move directly to next calibration stage, rerun through loop first

                case 1: // Calibration stage 1
                    telemetry.speak("Please position the ultrasonic sensor exactly 1 meter away from a wall, and press A.");

                    if (gamepad1.a && !aPressed) { // If A button is pressed and debounced
                        secondVoltage = ultrasonic0.getVoltage(); // Read second voltage and store value
                        aPressed = true; // Engage debouncing
                        calibrationStage = 2; // Advance to calibration stage 2
                    } // End if

                    break; // Don't move directly to next calibration stage, rerun through loop first

                case 2: // Calibration stage 2
                    // Calculate factor for converting voltage to distance, using standard slope formula
                    // Slope = (y2 - y1) / (x2 - x1) where y2 and y1 are distances, and x2 and x1 are respective voltages
                    // Even though voltage is the dependent variable in calibration, we want voltage to be independent
                    calculatedFactor = ((1 - 0.5) / (secondVoltage - firstVoltage)); // Calculate factor and store value

                    telemetry.speak("Calibration complete! Factor for converting voltage to distance is " + calculatedFactor);
                    telemetry.speak("Probably won't need this, but factor for converting distance to voltage is " + (1 / (calculatedFactor)));
                    telemetry.speak("Press A to restart calibration if needed.");

                    if (gamepad1.a && !aPressed) { // If A button is pressed and debounced
                        calibrationStage = 0; // Reset calibration stage to 0
                        aPressed = true; // Engage debouncing
                    } // End if
                    // Don't need to break for last case
            } // End switch case

            telemetry.addData("Calibration Stage:", calibrationStage);
            telemetry.addData("Ultrasonic0 Voltage:", ultrasonic0.getVoltage());
            telemetry.update();
        } // End while loop
    } // End OpMode void
} // End class Ultrasonic_test