package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

public class ArduinoInterface {
    private DigitalChannel pin0;
    private DigitalChannel pin1;
    private DigitalChannel pin2;
    private DigitalChannel pin3;
    private DigitalChannel pin4;
    private DigitalChannel pin5;
    private DigitalChannel pin6;
    private DigitalChannel pin7;

    public void init(HardwareMap hardwareMap) {
        pin0 = hardwareMap.get(DigitalChannel.class, "pin0");
        pin1 = hardwareMap.get(DigitalChannel.class, "pin1");
        pin2 = hardwareMap.get(DigitalChannel.class, "pin2");
        pin3 = hardwareMap.get(DigitalChannel.class, "pin3");
        pin4 = hardwareMap.get(DigitalChannel.class, "pin4");
        pin5 = hardwareMap.get(DigitalChannel.class, "pin5");
        pin6 = hardwareMap.get(DigitalChannel.class, "pin6");
        pin7 = hardwareMap.get(DigitalChannel.class, "pin7");
        pin0.setMode(DigitalChannel.Mode.INPUT);
        pin1.setMode(DigitalChannel.Mode.INPUT);
        pin2.setMode(DigitalChannel.Mode.INPUT);
        pin3.setMode(DigitalChannel.Mode.INPUT);
        pin4.setMode(DigitalChannel.Mode.INPUT);
        pin5.setMode(DigitalChannel.Mode.INPUT);
        pin6.setMode(DigitalChannel.Mode.INPUT);
        pin7.setMode(DigitalChannel.Mode.INPUT);
    }

    public void waitUntilReady() {
        while (!pin1.getState()) {}
        return;
    }

    public List<Integer> getState() {
        boolean[] pinStates = new boolean[8];
        DigitalChannel[] pins = {pin0, pin1, pin2, pin3, pin4, pin5, pin6, pin7};
        for (int i = 0; i < 8; i++) {
            pinStates[i] = pins[i].getState();
        }

        /*for (int i = 0; i < 8; i++) {
            pinStates[i] = true;
        }*/
        List<Integer> indexList = new ArrayList<>();
        for (int i = 0; i < 8; i++) {
            if (pinStates[i] && i != 1) {
                if (pinStates[i] && i == 7) {
                    indexList.add(1);
                }
                else {indexList.add(i);}
            }
        }
        return indexList;
    }

    public boolean[] getRawPins() {
        boolean[] pinStates = new boolean[8];
        DigitalChannel[] pins = {pin0, pin1, pin2, pin3, pin4, pin5, pin6, pin7};
        for (int i = 0; i < 8; i++) {
            pinStates[i] = pins[i].getState();
        }
        return pinStates;
    }
}
