package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.hardware.usb.UsbManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Arduino Serial Test", group = "Test")
public class SerialOverUsbTest extends OpMode implements SignalReader {

    private ElapsedTime runtime = new ElapsedTime();
    UsbSerialReader reader = new UsbSerialReader();

    // Instance variables to hold the latest values.
    private int lastSelection = -1;  // Default or invalid value
    private byte lastButtonCommand = -1;  // Default or invalid value

    @Override
    public void init() {
        UsbManager usbManager = (UsbManager) hardwareMap.appContext.getSystemService(Context.USB_SERVICE);
        reader.setReceiver(this);
        reader.initialize(usbManager);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Last Button Command: ", String.format("0x%02X", lastButtonCommand));
        telemetry.addData("Last Selection: ", lastSelection);
    }

    @Override
    public void stop() {
        reader.shutdown();
    }
    public void confirmSelection(int selection) {
        lastSelection = selection;
        lastButtonCommand = 0x20;
    }
    public void otherSignal(byte header) {
        lastButtonCommand = header;
    }
}
