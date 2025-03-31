package org.firstinspires.ftc.teamcode;

import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.util.Log;

import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;
import com.hoho.android.usbserial.util.SerialInputOutputManager;

import java.io.IOException;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

// This class encapsulates the USB serial functionality.
public class UsbSerialReader {

    public enum LedMode { ON, OFF, BLINK }
    public enum LedName { CONFIRM, START, ABORT }

    // Define your confirm header and the expected packet length.
    private static final byte CONFIRM_HEADER = 0x20;
    private static final int CONFIRM_PACKET_LENGTH = 3; // 1 header + 2 bytes
    private static final String TAG = "UsbSerialReader";

    private UsbSerialPort port;
    private SerialInputOutputManager usbIoManager;
    private ExecutorService executor;
    private SignalReader signalReceiver;

    // Call this method from your op mode's init() routine,
    // providing the UsbManager (from the Android context).
    public void initialize(UsbManager usbManager) {
        // Probe for USB serial drivers connected to the device.
        List<UsbSerialDriver> availableDrivers = UsbSerialProber.getDefaultProber().findAllDrivers(usbManager);
        if (availableDrivers.isEmpty()) {
            Log.d(TAG, "No USB serial drivers found.");
            return;
        }

        // Open the first available driver.
        UsbSerialDriver driver = availableDrivers.get(0);
        port = driver.getPorts().get(0);

        // Open a connection to the USB device.
        try {
            // Store the UsbDevice instance in a variable.
            UsbDeviceConnection device = usbManager.openDevice(driver.getDevice());
            if (device == null) {
                Log.e(TAG, "Device could not be opened. Check permissions.");
                return;
            }
            // Open the port with the same device instance.
            port.open(device);
            // Set the port parameters to match the Arduino (9600 baud, 8 data bits, 1 stop bit, no parity)
            port.setParameters(9600, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);
        } catch (IOException e) {
            Log.e(TAG, "Error opening USB port: " + e.getMessage());
            return;
        }

        // Create an IO manager to listen for data.
        usbIoManager = new SerialInputOutputManager(port, new SerialInputOutputManager.Listener() {
            @Override
            public void onNewData(final byte[] data) {
                handleIncomingData(data);
            }

            @Override
            public void onRunError(Exception e) {
                Log.e(TAG, "Runner stopped: " + e.getMessage());
            }
        });

        // Start the IO manager in a background thread.
        executor = Executors.newSingleThreadExecutor();
        usbIoManager.start();
    }

    // When processing data:
    private void handleIncomingData(byte[] data) {
        if (data == null || data.length < 1) return;

        if (data[0] == CONFIRM_HEADER) {
            if (data.length >= CONFIRM_PACKET_LENGTH) {
                byte lowByte = data[1];
                byte highByte = data[2];
                int selectionData = ((highByte & 0xFF) << 8) | (lowByte & 0xFF);
                Log.d(TAG, "Confirm packet received. Bitmask: " + Integer.toBinaryString(selectionData));

                if (signalReceiver != null) {
                    signalReceiver.confirmSelection(selectionData);
                }
            } else {
                Log.w(TAG, "Incomplete confirm packet received.");
            }
        } else {
            Log.d(TAG, "Other signal received. Header: " + data[0]);
            if (signalReceiver != null) {
                signalReceiver.otherSignal(data[0]);
            }
        }
    }

    // Call this when you want to stop reading and close the port.
    public void shutdown() {
        if (usbIoManager != null) {
            usbIoManager.stop();
        }
        if (port != null) {
            try {
                port.close();
            } catch (IOException e) {
                // Ignore errors on close.
            }
        }
        if (executor != null) {
            executor.shutdownNow();
        }
    }

    public void setReceiver(SignalReader receiver) {
        this.signalReceiver = receiver;
    }

    public void sendLedCommand(LedMode ledState, LedName buttonName) {
        byte command = -1; // If not set, nothing will happen.
        switch (ledState) {
            case ON:
                command = 0x30;
                break;
            case OFF:
                command = 0x31;
                break;
            case BLINK:
                command = 0x32;
                break;
        }
        byte buttonIndex = -1; // If not set, nothing will happen.
        switch (buttonName) {
            case CONFIRM:
                buttonIndex = 0x00;
                break;
            case START:
                buttonIndex = 0x01;
                break;
            case ABORT:
                buttonIndex = 0x02;
                break;
        }

        // Only attempt to write if the port is available.
        if (port == null) {
            Log.e(TAG, "USB port is not initialized.");
            return;
        }
        try {
            byte[] packet = {command, buttonIndex};
            port.write(packet, 100);
        } catch (IOException e) {
            Log.e(TAG, "Failed to send LED command: " + e.getMessage());
        }
    }
}
