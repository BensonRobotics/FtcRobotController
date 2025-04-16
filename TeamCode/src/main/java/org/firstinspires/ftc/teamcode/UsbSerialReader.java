/**
 * Bit position to selection map:
 * 0 = Vanilla
 * 1 = Chocolate
 * 2 = Strawberry
 * 3 = Middle slightly right
 * 4 = Middle far right
 * 5 = Middle slightly left (buggy for some reason)
 * 6 = Middle
 * 7 =
 * 8 =
 * 9 =
 *
 */


package org.firstinspires.ftc.teamcode;

import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.util.Log;

import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;
import com.hoho.android.usbserial.util.SerialInputOutputManager;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.IOException;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

// This class encapsulates the USB serial functionality.
public class UsbSerialReader {
    // Define your confirm header and the expected packet length.
    private static final byte CONFIRM_HEADER = 0x20;
    private static final int CONFIRM_PACKET_LENGTH = 3; // 1 header + 2 bytes
    private static final String TAG = "UsbSerialReader";

    private UsbSerialPort port;
    private SerialInputOutputManager usbIoManager;
    private ExecutorService executor;
    private SignalReader signalReceiver;
    private ElapsedTime ignoreAfterFullPacketTimer = new ElapsedTime();

    // Call this method from your op mode's init() routine,
    // providing the UsbManager (from the Android context).
    public void initialize(UsbManager usbManager) {
        ignoreAfterFullPacketTimer.reset();
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
            // Set the port parameters to match the Arduino (110 baud, 8 data bits, 2 stop bits, no parity)
            port.setParameters(9600, 8, UsbSerialPort.STOPBITS_2, UsbSerialPort.PARITY_NONE);
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

        if (data[0] == CONFIRM_HEADER && ignoreAfterFullPacketTimer.milliseconds() > 1000) {
            if (data.length >= CONFIRM_PACKET_LENGTH && (data[1] != 0 || (data[2] & 0xC0) != 0)) {
                byte lowByte = data[1];
                byte highByte = data[2];
                int selectionData = ((highByte & 0xC0) << 8) | (lowByte & 0xFF);
                Log.d(TAG, "Confirm packet received. Bitmask: " + Integer.toBinaryString(selectionData));
                ignoreAfterFullPacketTimer.reset();
                if (signalReceiver != null) {
                    signalReceiver.confirmSelection(selectionData);
                }
            } else {
                Log.w(TAG, "Incomplete confirm packet received.");
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
}
