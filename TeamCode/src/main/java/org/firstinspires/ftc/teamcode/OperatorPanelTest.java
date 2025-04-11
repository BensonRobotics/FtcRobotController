package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@TeleOp(name = "Operator Panel Test", group = "Test")
public class OperatorPanelTest extends OpMode {

    private final String[] operatorButtonNames = {"startButton", "abortButton", "resetButton"};
    private final String[] operatorLedNames = {"startLed", "abortLed", "resetLed"};
    private DigitalChannel[] operatorButtons = new DigitalChannel[3]; // 3 op buttons
    private DigitalChannel[] operatorLeds = new DigitalChannel[3]; // 3 op LEDs
    private boolean[] lastButtonStates = new boolean[operatorButtons.length];
    // All will be set to true in setup
    private ElapsedTime debounceTimer = new ElapsedTime();

    @Override
    public void init() {
        for (int i = 0; i < operatorButtons.length; i++) {
            operatorButtons[i] = hardwareMap.get(DigitalChannel.class, operatorButtonNames[i]);
            operatorButtons[i].setMode(DigitalChannel.Mode.INPUT);
        }
        for (int i = 0; i < operatorLeds.length; i++) {
            operatorLeds[i] = hardwareMap.get(DigitalChannel.class, operatorLedNames[i]);
            operatorLeds[i].setMode(DigitalChannel.Mode.OUTPUT);
            operatorLeds[i].setState(true);
        }
        Arrays.fill(lastButtonStates, true);
    }

    @Override
    public void start() {
        debounceTimer.reset();
    }

    @Override
    public void loop() {
        for (int i = 0; i < operatorButtons.length; i++) {
            if (!operatorButtons[i].getState()) {
                if (!lastButtonStates[i] && debounceTimer.milliseconds() > 20) { // If first time pressed since last
                    lastButtonStates[i] = true;
                    operatorLeds[i].setState(!operatorLeds[i].getState());
                    debounceTimer.reset();
                }
            } else {
                lastButtonStates[i] = false;
            }
        }
    }

    @Override
    public void stop() {
        for (DigitalChannel operatorLed : operatorLeds) {
            operatorLed.setState(true);
        }
    }
}
