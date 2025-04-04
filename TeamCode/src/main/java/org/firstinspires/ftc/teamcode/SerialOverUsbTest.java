package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.hardware.usb.UsbManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "Arduino Serial Test", group = "Test")
public class SerialOverUsbTest extends OpMode implements SignalReader {

    private ElapsedTime runtime = new ElapsedTime();
    UsbSerialReader reader = new UsbSerialReader();

    // Instance variables to hold the latest values.
    private int lastSelection = -1;  // Default or invalid value
    private List<Integer> lastSchedule = new ArrayList<>();
    private float lastCost;
    private String lastFlavor;
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
        telemetry.addData("Toppings: ", lastSchedule.toString());
                telemetry.addData("Flavor: ", lastFlavor);
                telemetry.addData("Price: ", String.format(
                        Locale.US, "%.2f$", lastCost));
        telemetry.update();
    }

    @Override
    public void stop() {
        reader.shutdown();
    }
    public void confirmSelection(int selection) {
        List<Integer> newSchedule = new ArrayList<>();
        float newCost = 0.00f;
        String newFlavor = "None";

        // Select flavors
        for (int i = 0; i < 3; i++) {
            if (((selection >> i) & 0x01) == 1) { // If flavor selected
                switch (i) { // Which one
                    case 0: newFlavor = "Vanilla"; break;
                    case 1: newFlavor = "Chocolate"; break;
                    case 2: newFlavor = "Strawberry"; break;
                    default: break; // Still "None"
                }
                break; // Breaks the for loop, only one flavor allowed
            }
        }
        lastFlavor = newFlavor;

        for (int i = 3; i < 10; i++) {
            if (((selection >> i) & 0x01) == 1) {
                newSchedule.add(i - 3);
            }
        }
        lastSchedule = newSchedule;

        // First topping is free, only if with ice cream
        newCost += 0.50f * newSchedule.size(); // Each is 50 cents
        if (!newFlavor.equals("None")) {
            newCost += 2.00f; // Bowl cost
            if (!newSchedule.isEmpty()) { newCost -= 0.50f; }
            // Only apply discount if you have ordered ice cream and at least 1 topping
        }
        lastCost = newCost; // Save cost to queue
    }
}
