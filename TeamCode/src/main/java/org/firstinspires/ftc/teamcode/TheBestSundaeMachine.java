package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.content.Context;
import android.hardware.usb.UsbManager;
import java.util.Locale;
import java.util.List;
import java.util.ArrayList;

@TeleOp(name = "Sundae Machine over USB, Hurray!", group = "Off-Season")
public class TheBestSundaeMachine extends LinearOpMode implements SignalReader {

    // Timers
    private ElapsedTime toppingFallTimer = new ElapsedTime();

    // Constants
    private static final Locale LOCALE = Locale.US;
    private static final float BASE_PRICE = 2.00F;
    private static final int NUM_OF_TOPPINGS = 7;
    private static final int MAX_QUEUE_TELEMETRY = 3;
    private static final int TOPPING_FALL_WAIT = 1000; // 1 second delay
    private static final double DISPENSER_POWER = 0.5;

    // Positions and loads (placeholders, update as needed)
    private static final int END_POSITION = 8000;
    // Make sure these are all the same length as NUM_OF_TOPPINGS
    private final int[] BOWL_POSITIONS = {1000, 2000, 3000, 4000, 5000, 6000, 7000};
    private final int[] DISPENSER_SECTORS = {8, 8, 8, 8, 8, 8, 8};
    private final int[] SECTORS_PER_DISPENSE = {1, 1, 1, 1, 1, 1, 1};
    private int[] dispenserTally = {0, 0, 0, 0, 0, 0, 0};

    // Motors
    private DcMotorEx conveyorMotor;
    private final String[] allMotorNames = {"topping0Motor", "topping1Motor", "topping2Motor",
            "topping3Motor", "topping4Motor", "topping5Motor", "topping6Motor", "conveyorMotor"};
    private DcMotorEx[] allMotors = new DcMotorEx[allMotorNames.length];

    // Topping schedule and current topping
    private List<List<Integer>> scheduleQueue = new ArrayList<>();
    private List<String> flavorQueue = new ArrayList<>();
    private List<Float> costQueue = new ArrayList<>();
    private List<Integer> currentSchedule = new ArrayList<>();
    private boolean isEmergencyStopped = false;

    // State management using enum
    private enum MachineState {
        IDLE,
        TARGET_DISPENSER,
        TRAVELLING,
        DISPENSING,
        WAIT_FOR_TOPPING_FALL,
        FINISHED
    }
    private MachineState machineState = MachineState.IDLE;

    UsbSerialReader reader = new UsbSerialReader();

    @Override
    public void runOpMode() throws InterruptedException {
        UsbManager usbManager = (UsbManager) hardwareMap.appContext.getSystemService(Context.USB_SERVICE);
        reader.setReceiver(this);
        reader.initialize(usbManager);
        // Initialize motors
        for (int i = 0; i < allMotors.length; i++) {
            allMotors[i] = hardwareMap.get(DcMotorEx.class, allMotorNames[i]);
            allMotors[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            allMotors[i].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            allMotors[i].setTargetPosition(0);
            allMotors[i].setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            allMotors[i].setPower(DISPENSER_POWER);
        }
        // Increase conveyor motor power
        conveyorMotor = allMotors[allMotors.length-1];
        conveyorMotor.setPower(1);

        telemetry.addLine("Initialized!");
        telemetry.update();
        waitForStart();

        // Reset timers
        toppingFallTimer.reset();

        while (opModeIsActive()) {
            // State machine
            switch (machineState) {
                case IDLE:
                    break; // Nothing
                case TARGET_DISPENSER:
                    if (!currentSchedule.isEmpty()) {
                        conveyorMotor.setTargetPosition(BOWL_POSITIONS[currentSchedule.get(0)]);
                        machineState = MachineState.TRAVELLING;
                    } else {
                        machineState = MachineState.FINISHED;
                    }
                    break;
                case TRAVELLING:
                    if (!conveyorMotor.isBusy()) {
                        dispenseTopping(currentSchedule.get(0));
                        machineState = MachineState.DISPENSING;
                    }
                    break;
                case DISPENSING:
                    if (!allMotors[currentSchedule.get(0)].isBusy()) {
                        toppingFallTimer.reset();
                        machineState = MachineState.WAIT_FOR_TOPPING_FALL;
                    }
                    break;
                case WAIT_FOR_TOPPING_FALL:
                    if (toppingFallTimer.milliseconds() > TOPPING_FALL_WAIT) {
                        // Done dispensing
                        currentSchedule.remove(0);
                        machineState = MachineState.TARGET_DISPENSER;
                    }
                    break;
                case FINISHED:
                    // scheduleQueue now has an empty schedule on top
                    // It will get removed when reset button is pressed
                    conveyorMotor.setTargetPosition(END_POSITION);
                    break;
            }
            if (!costQueue.isEmpty()) { // If queue is empty, only need to check one of them
                for (int i = 0; i < Math.min(costQueue.size(), MAX_QUEUE_TELEMETRY); i++) {
                    String suffix =
                            (i+1 == 1) ? "st" :
                            (i+1 == 2) ? "nd" :
                            (i+1 == 3) ? "rd" : "th";
                    telemetry.addLine((i+1) + suffix + " Order: ");
                    telemetry.addData("Flavor: ", flavorQueue.get(i));
                    telemetry.addData("Price: ", String.format(
                            LOCALE, "%.2f$", costQueue.get(i)));
                    telemetry.addLine();
                }
            } else { // If queue is empty
                telemetry.addLine("Queue is Empty.");
            }
            telemetry.update();
        }
        reader.shutdown();
    }

    /**
     * Builds a schedule of toppings based on the read input.
     * The start button is on index 0; toppings are from index 1 onward.
     */
    public void processSelection(int selection) {
        // This is designed to allow customers to go back and add toppings when their order is done.
        // There are two options: one where the customer can go back and add toppings,
        // and one where multiple customers can order in a queue system. I should make a poll on that.
        List<Integer> newSchedule = new ArrayList<>();
        float newCost = BASE_PRICE;
        String newFlavor = "None";
            for (int i = 0; i < NUM_OF_TOPPINGS; i++) {
                if (((selection >> i) & 0x01) == 1) {
                    newSchedule.add(i);
                }
            }
            scheduleQueue.add(newSchedule);

        // First topping is free
            float toppingCost = 0.50f * (Math.max((float) newSchedule.size(), 1.00f) - 1.00f);
            // If cost is zero, assume bowl has not been paid for yet, so add it
            newCost += toppingCost;
            costQueue.add(newCost);

            for (int i = NUM_OF_TOPPINGS; i < NUM_OF_TOPPINGS + 3; i++) {
                if (((selection >> i) & 0x01) == 1) { // If flavor selected
                    switch (i - NUM_OF_TOPPINGS) { // Which one
                        case 0: newFlavor = "Vanilla"; break;
                        case 1: newFlavor = "Chocolate"; break;
                        case 2: newFlavor = "Strawberry"; break;
                        default: break; // No change
                    }
                    break; // Breaks the for loop, only one flavor at a time
                }
            }
            flavorQueue.add(newFlavor);

            if (machineState == MachineState.IDLE) {
                reader.sendLedCommand(UsbSerialReader.LedMode.ON, UsbSerialReader.LedName.START);
            }
    }

    /**
     * Dispenses a topping by calculating the target position.
     *
     * @param topping the topping index (1-indexed)
     */
    private void dispenseTopping(int topping) {
        dispenserTally[topping] += SECTORS_PER_DISPENSE[topping];
        double ticksPerLoad = 2786.2 / DISPENSER_SECTORS[topping];
        int dispenserTarget = (int) (dispenserTally[topping] * ticksPerLoad);
        allMotors[topping].setTargetPosition(dispenserTarget);
    }

    public void startCycle() {
        if (!scheduleQueue.isEmpty()) {
            machineState = MachineState.TARGET_DISPENSER;
            costQueue.remove(0);
            flavorQueue.remove(0);
            currentSchedule = scheduleQueue.remove(0);
        }
        if (isEmergencyStopped) { // Only reset powers if emergency stopped
            for (DcMotorEx motor : allMotors) {
                motor.setPower(DISPENSER_POWER);
            }
            conveyorMotor.setPower(1);
            isEmergencyStopped = false;
        }
        reader.sendLedCommand(UsbSerialReader.LedMode.ON, UsbSerialReader.LedName.ABORT);
    }

    public void emergencyStop() {
        for (DcMotorEx motor : allMotors) {
            motor.setPower(0); // Kill power to all motors
        }
        isEmergencyStopped = true; // Flag for emergency stop
        reader.sendLedCommand(UsbSerialReader.LedMode.BLINK, UsbSerialReader.LedName.ABORT);
    }

    public void resetSystem() {
        if (!scheduleQueue.isEmpty()) {
            // Clear the top of the queue
            scheduleQueue.remove(0);
            reader.sendLedCommand(UsbSerialReader.LedMode.ON, UsbSerialReader.LedName.START);
        } else {
            reader.sendLedCommand(UsbSerialReader.LedMode.OFF, UsbSerialReader.LedName.START);
        }
        machineState = MachineState.IDLE;
        conveyorMotor.setTargetPosition(0);
        if (isEmergencyStopped) { // Only reset powers if emergency stopped
            for (DcMotorEx motor : allMotors) {
                motor.setPower(DISPENSER_POWER);
            }
            conveyorMotor.setPower(1);
            isEmergencyStopped = false;
        }
        reader.sendLedCommand(UsbSerialReader.LedMode.ON, UsbSerialReader.LedName.ABORT);
    }

    public void confirmSelection(int selection) {
        processSelection(selection);
    }
    public void otherSignal(byte header) {
        switch (header) {
            case 0x10: // Start header
                startCycle();
                break;
            case 0x11: // Abort header
                emergencyStop();
                break;
            case 0x12: // Reset header
                resetSystem();
                break;
            }
        }
}

