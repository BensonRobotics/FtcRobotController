package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImplOnSimple;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArduinoI2CDriver {

    private final I2cDeviceSynch deviceSynch;
    private static final int INPUT_REGISTER = 0x00;
    private static final int OUTPUT_REGISTER = 0x01;
    // Constructor: initialize the device using hardwareMap and device name
    public ArduinoI2CDriver(HardwareMap hardwareMap, String deviceName) {
        I2cDevice arduinoDevice = hardwareMap.get(I2cDevice.class, deviceName);
        // The cast is necessary since the device implements I2cDeviceSynchSimple
        deviceSynch = new I2cDeviceSynchImplOnSimple((I2cDeviceSynchSimple) arduinoDevice, false);
    }

    // Call this method from init() in your OpMode
    public void engage() {
        deviceSynch.engage();
    }

    // Read the pin states from the Arduino.
    // Assumes that the Arduino returns a single byte where each bit represents a pin state.
    public int readButtonPins() {
        byte[] readBuffer = deviceSynch.read(INPUT_REGISTER, 2);
        int pinStates = ((readBuffer[0] & 0xFF) << 8) | (readBuffer[1] & 0xFF);
        return pinStates & 0x0FFF;
    }

    public boolean[] readButtonPinsAsBooleans() {
        int pinStates = readButtonPins();
        boolean[] pins = new boolean[12];
        // Each bit (0-11) represents one pin's state.
        for (int i = 0; i < 12; i++) {
            pins[i] = ((pinStates >> i) & 1) == 1;
        }
        return pins;
    }

    public void enableResetRelay(boolean state) {
        byte command = state ? (byte) 1 : (byte) 0;
        deviceSynch.write(OUTPUT_REGISTER, new byte[] { command });
    }

    // Clean up the connection
    public void close() {
        if (deviceSynch != null) {
            deviceSynch.close();
        }
    }
}
