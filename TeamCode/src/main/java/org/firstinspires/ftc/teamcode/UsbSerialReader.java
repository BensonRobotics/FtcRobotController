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

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.stream.IntStream;

// This class encapsulates the USB serial functionality.
public class UsbSerialReader {
    // Define your confirm header and the expected packet length.
    private static final String TAG = "UsbSerialReader";

    private UsbSerialPort port;
    private SerialInputOutputManager usbIoManager;
    private ExecutorService executor;
    private SignalReader signalReceiver;
    private final ElapsedTime ignoreAfterFullPacketTimer = new ElapsedTime();
    private ByteArrayOutputStream recvBuffer = new ByteArrayOutputStream();


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
                try {
                    recvBuffer.write(data);
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }
                extractFrames();
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

    private void extractFrames() {
        int idx;

        // Keep extracting as long as we find a 0x00 delimiter
        while (true) {
            // 1) Fresh snapshot each iteration (effectively final for lambda)
            final byte[] finalAll = recvBuffer.toByteArray();

            // 2) Find the COBS delimiter in this snapshot
            idx = IntStream.range(0, finalAll.length)
                    .filter(i -> finalAll[i] == 0x00)
                    .findFirst()
                    .orElse(-1);

            // 3) No delimiter? we’re done
            if (idx < 0) break;

            // 4) Extract the frame bytes (everything before the 0x00)
            byte[] frame = Arrays.copyOf(finalAll, idx);

            // 5) Remove processed data from recvBuffer
            recvBuffer.reset();
            recvBuffer.write(finalAll, idx + 1, finalAll.length - (idx + 1));

            // 6) Decode and verify checksum/mask as before
            byte[] payload = decodeCOBS(frame);
            if (payload.length == 3) {
                int low  = payload[0] & 0xFF;
                int high = payload[1] & 0xFF;
                int cs   = payload[2] & 0xFF;

                if (cs == (low ^ high) && ((low | (high & 0xC0)) != 0)) {
                    int order = (high << 8) | low;
                    signalReceiver.confirmSelection(order);
                    break;  // stop after a valid frame
                }
            }
            // Loop back to catch any additional frames in the newly updated buffer
        }
    }


    public static byte[] decodeCOBS(byte[] data) throws IllegalArgumentException {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        int i = 0;
        while (i < data.length) {
            int code = data[i++] & 0xFF;
            if (code == 0 || i + code - 1 > data.length) throw new IllegalArgumentException("Bad COBS");
            for (int j = 1; j < code; j++) out.write(data[i++]);
            if (code < 0xFF && i < data.length) out.write(0);
        }
        return out.toByteArray();
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

    private int indexOf(byte[] array, byte value) {
        for (int i = 0; i < array.length; i++) {
            if (array[i] == value) return i;
        }
        return -1;
    }
}
