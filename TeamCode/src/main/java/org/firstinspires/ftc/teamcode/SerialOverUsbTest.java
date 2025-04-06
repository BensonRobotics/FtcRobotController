package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.hardware.usb.UsbManager;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "Arduino Serial Test", group = "Test")
public class SerialOverUsbTest extends OpMode implements SignalReader {

    private ElapsedTime runtime = new ElapsedTime();
    UsbSerialReader reader = new UsbSerialReader();

    // Instance variables to hold the latest values.
    private List<List<Integer>> scheduleQueue = new ArrayList<>();
    private List<String> flavorQueue = new ArrayList<>();
    private List<Float> costQueue = new ArrayList<>();
    private int lastQueueLength = 0;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        clearGodDamnit();
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
        clearGodDamnit();
    }

    @Override
    public void loop() {
        if (lastQueueLength != scheduleQueue.size()) {
            clearGodDamnit();
            lastQueueLength = scheduleQueue.size();
        }
        if (!costQueue.isEmpty()) { // If queue is empty, only need to check one of them
            for (int i = 0; i < costQueue.size(); i++) {
                String orderInfo = String.format(Locale.US, "$%.2f", costQueue.get(i))+", "+
                        flavorQueue.get(i)+", "+toppingStringFromList(scheduleQueue.get(i));
                String suffix =
                        (i+1 == 1) ? "st" :
                                (i+1 == 2) ? "nd" :
                                        (i+1 == 3) ? "rd" : "th";
                telemetry.addData((i+1)+suffix+" Order Info", orderInfo);
            }
        } else { // If queue is empty
            telemetry.addData("Queue Status", "Empty.");
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        reader.shutdown();
        clearGodDamnit();
    }
    public void confirmSelection(int selection) {
        List<Integer> newSchedule = new ArrayList<>();
        float newCost = 0.00f;
        String newFlavor = "No Ice Cream";

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
        flavorQueue.add(newFlavor);

        for (int i = 3; i < 10; i++) {
            if (((selection >> i) & 0x01) == 1) {
                newSchedule.add(i - 3);
            }
        }
        scheduleQueue.add(newSchedule);

        // First topping is free, only if with ice cream
        newCost += 0.50f * newSchedule.size(); // Each is 50 cents
        if (!newFlavor.equals("No Ice Cream")) {
            newCost += 2.00f; // Bowl cost
            if (!newSchedule.isEmpty()) { newCost -= 0.50f; }
            // Only apply discount if you have ordered ice cream and at least 1 topping
        }
        costQueue.add(newCost); // Save cost to queue
    }

    public String toppingStringFromList(List<Integer> schedule) {
        if (!schedule.isEmpty()) {
            return schedule.toString();
        } else {
            return "No Toppings";
        }
    }

    public void clearGodDamnit() {
        for (int i = 0; i < 2; i++) {
            telemetry.clearAll();
        }
    }
}
