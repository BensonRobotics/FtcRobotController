package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.hardware.usb.UsbManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Arduino Serial Test", group = "Test")
public class SerialOverUsbTest extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    UsbSerialReader reader = new UsbSerialReader();

    @Override
    public void init() {
        UsbManager usbManager = (UsbManager) hardwareMap.appContext.getSystemService(Context.USB_SERVICE);
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
    }

    @Override
    public void stop() {
        reader.shutdown();
    }

    public class Receiver {
        public void confirmSelection(int selection) {

        }
        public void otherSignal(byte header) {

        }
    }
}
