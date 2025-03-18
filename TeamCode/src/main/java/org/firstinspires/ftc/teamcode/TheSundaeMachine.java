package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Sundae OpMode", group = "Off-Season")
public class TheSundaeMachine extends OpMode {

    private ArduinoI2CDriver arduinoDriver;
    private ElapsedTime runtime = new ElapsedTime();

    /**
     * This method will be called once, when the INIT button is pressed.
     */
    @Override
    public void init() {
        arduinoDriver = new ArduinoI2CDriver(hardwareMap, "arduino");
        arduinoDriver.engage();
        telemetry.addData("Status", "I2C device engaged");
        telemetry.addData("Status", "Initialized");
    }

    /**
     * This method will be called repeatedly during the period between when
     * the INIT button is pressed and when the START button is pressed (or the
     * OpMode is stopped).
     */
    @Override
    public void init_loop() {
    }

    /**
     * This method will be called once, when the START button is pressed.
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /**
     * This method will be called repeatedly during the period between when
     * the START button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {
        // Read the 12 input pins as a boolean array.
        boolean[] buttonPins = arduinoDriver.readButtonPinsAsBooleans();

        // Display each input pin state via telemetry.
        StringBuilder pinStates = new StringBuilder();
        for (int i = 0; i < buttonPins.length; i++) {
            pinStates.append("Pin ").append(i).append(": ").append(buttonPins[i] ? "HIGH" : "LOW").append("  ");
        }
        telemetry.addData("Input Pins", pinStates.toString());

        // Example: Toggle the output pin based on gamepad1.x.
        boolean outputState = gamepad1.x;
        arduinoDriver.enableResetRelay(outputState);
        telemetry.addData("Output Pin", outputState ? "HIGH" : "LOW");

        telemetry.update();
    }

    /**
     * This method will be called once, when this OpMode is stopped.
     * <p>
     * Your ability to control hardware from this method will be limited.
     */
    @Override
    public void stop() {
        arduinoDriver.close();
    }
}
