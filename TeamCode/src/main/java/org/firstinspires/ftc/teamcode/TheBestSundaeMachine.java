package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.content.Context;
import android.hardware.usb.UsbManager;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import java.util.Arrays;
import java.util.Collections;
import java.util.Locale;
import java.util.List;
import java.util.ArrayList;

@TeleOp(name = "Sundae Machine over USB, Hurray!", group = "Off-Season")
public class TheBestSundaeMachine extends LinearOpMode implements SignalReader {

    // Timers
    private ElapsedTime toppingFallTimer = new ElapsedTime();
    private ElapsedTime creamDispenseTimer = new ElapsedTime();

    // Constants
    private static final Locale LOCALE = Locale.US;
    private static final int NUM_OF_TOPPINGS = 7;
    private static final int NUM_OF_FLAVORS = 3;
    private static final int TOPPING_FALL_WAIT = 1000; // 1 second delay
    private static final double DISPENSER_POWER = 0.5;

    // Positions and loads (placeholders, update as needed)
    // Make sure these are all the same length as NUM_OF_TOPPINGS
    private final int[] BUTTON_TO_TOPPING_NUM = {0, 1, 2, 3, 4, 5, 6};
    private final int[] BOWL_POSITIONS = {1000, 2000, 3000, 4000, 5000, 6000, 7000};
    private final int[] DISPENSER_SECTORS = {8, 8, 8, 8, 8, 8, -1}; // -1 is servo, invalid
    private final int[] SECTORS_PER_DISPENSE = {1, 1, 1, 1, 1, 1, -1}; // Same
    private final int CREAM_DISPENSE_DURATION = 1500;
    private final float CREAM_DISPENSE_ANGLE = 0.25f;
    private int[] dispenserTally = {0, 0, 0, 0, 0, 0, -1}; // Same

    // Motors
    private DcMotorEx conveyorMotor;
    private Servo creamServo;
    private final String[] allMotorNames = {"topping0Motor", "topping1Motor", "topping2Motor",
            "topping3Motor", "topping4Motor", "topping5Motor", "T6PLACEHOLDER", "conveyorMotor"};
    // topping6 is a servo
    private final int SERVO_INDEX = 6;
    private DcMotorEx[] allMotors = new DcMotorEx[allMotorNames.length];

    private DigitalChannel minEndstop;
    private DigitalChannel maxEndstop;

    private final String[] operatorButtonNames = {"startButton", "abortButton", "resetButton"};
    private final String[] operatorLedNames = {"startLed", "abortLed", "resetLed"};
    private DigitalChannel[] operatorButtons = new DigitalChannel[3]; // 3 op buttons
    private DigitalChannel[] operatorLeds = new DigitalChannel[3]; // 3 op LEDs
    private LED testLed;

    // Topping schedule and current topping
    private List<List<Integer>> scheduleQueue = new ArrayList<>();
    private List<String> flavorQueue = new ArrayList<>();
    private List<Float> costQueue = new ArrayList<>();
    private List<Integer> currentSchedule = new ArrayList<>();
    private boolean isEmergencyStopped = false;
    private boolean[] lastButtonStates = new boolean[operatorButtons.length];
    // All will be set to true in setup
    private int lastQueueLength = 0;
    private LedState[] operatorLedStates = new LedState[] {
            LedState.OFF,
            LedState.ON,
            LedState.ON
    };
    private ElapsedTime[] ledBlinkTimers = new ElapsedTime[operatorLeds.length];
    private ElapsedTime debounceTimer = new ElapsedTime();

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
    private enum LedState { ON, OFF, BLINK }

    UsbSerialReader reader = new UsbSerialReader();

    @Override
    public void runOpMode() throws InterruptedException {
        for (int i = 0; i < ledBlinkTimers.length; i++) {
            ledBlinkTimers[i] = new ElapsedTime();
        }
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        clearGoshDarnit();
        UsbManager usbManager = (UsbManager) hardwareMap.appContext.getSystemService(Context.USB_SERVICE);
        reader.setReceiver(this);
        reader.initialize(usbManager);
        // Initialize motors, except servo
        for (int i = 0; i < allMotors.length; i++) {
            if (i != SERVO_INDEX) {
                allMotors[i] = hardwareMap.get(DcMotorEx.class, allMotorNames[i]);
                allMotors[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                allMotors[i].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                allMotors[i].setTargetPosition(0);
                allMotors[i].setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                allMotors[i].setPower(DISPENSER_POWER);
            } else { // Servo is not a motor
                allMotors[i] = null;
            }
        }
        creamServo = hardwareMap.get(Servo.class, "creamServo");
        // Increase conveyor motor power
        conveyorMotor = allMotors[allMotors.length-1];
        conveyorMotor.setPower(1);

        for (int i = 0; i < operatorButtons.length; i++) {
            operatorButtons[i] = hardwareMap.get(DigitalChannel.class, operatorButtonNames[i]);
            operatorButtons[i].setMode(DigitalChannel.Mode.INPUT);
        }
        for (int i = 0; i < operatorLeds.length; i++) {
            operatorLeds[i] = hardwareMap.get(DigitalChannel.class, operatorLedNames[i]);
            operatorLeds[i].setMode(DigitalChannel.Mode.OUTPUT);
        }

        Arrays.fill(lastButtonStates, true); // Set all button states to true
        // This is to ignore any buttons that are pressed during initialization

        minEndstop = hardwareMap.get(DigitalChannel.class, "minEndstop");
        maxEndstop = hardwareMap.get(DigitalChannel.class, "maxEndstop");
        minEndstop.setMode(DigitalChannel.Mode.INPUT);
        maxEndstop.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // Reset timers
        toppingFallTimer.reset();
        creamDispenseTimer.reset();
        debounceTimer.reset();
        for (ElapsedTime timer : ledBlinkTimers) {
            timer.reset();
        }

        while (opModeIsActive()) {

            // Check operator buttons and act ONCE if they are pressed and debounced
            // Remember that button presses are falling edge
            for (int i = 0; i < operatorButtons.length; i++) {
                if (!operatorButtons[i].getState() && debounceTimer.milliseconds() > 20) {
                    if (!lastButtonStates[i]) { // If first time pressed since last
                        lastButtonStates[i] = true;
                        operatorAction(i);
                        debounceTimer.reset();
                    }
                } else {
                    lastButtonStates[i] = false;
                }
            }

            // LEDs must be driven using transistors, as digital I/O has insufficient power
            for (int i = 0; i < operatorLeds.length; i++) {
                switch (operatorLedStates[i]) {
                    case ON: operatorLeds[i].setState(true);
                    break;
                    case OFF: operatorLeds[i].setState(false);
                    break;
                    case BLINK: if (ledBlinkTimers[i].milliseconds() > 250) {
                        operatorLeds[i].setState(!operatorLeds[i].getState());
                        ledBlinkTimers[i].reset();
                    }
                    break;
                }
            }

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
                        conveyorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        conveyorMotor.setPower(1);
                    }
                    break;
                case TRAVELLING:
                    if (!conveyorMotor.isBusy()) {
                        dispenseTopping(currentSchedule.get(0));
                        machineState = MachineState.DISPENSING;
                    }
                    break;
                case DISPENSING:
                    if (currentSchedule.get(0) != SERVO_INDEX) {
                        if (!allMotors[currentSchedule.get(0)].isBusy()) {
                            toppingFallTimer.reset();
                            machineState = MachineState.WAIT_FOR_TOPPING_FALL;
                        }
                    } else {
                        if (creamDispenseTimer.milliseconds() > CREAM_DISPENSE_DURATION) {
                            creamServo.setPosition(0);
                            toppingFallTimer.reset();
                            machineState = MachineState.WAIT_FOR_TOPPING_FALL;
                        }
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
                    break;
            }

            if (minEndstop.getState() && conveyorMotor.getVelocity() < -100) {
                conveyorMotor.setPower(0);
                conveyorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            } else if (maxEndstop.getState() && conveyorMotor.getVelocity() > 100) {
                conveyorMotor.setPower(0);
            }

            if (lastQueueLength != scheduleQueue.size()) {
                clearGoshDarnit();
                lastQueueLength = scheduleQueue.size();
            }
            if (!costQueue.isEmpty()) { // If queue is empty, only need to check one of them
                for (int i = 0; i < costQueue.size(); i++) {
                    String orderInfo = String.format(LOCALE, "$%.2f", costQueue.get(i))+", "+
                            flavorQueue.get(i);
                    String suffix =
                            (i+1 == 1) ? "st" :
                                    (i+1 == 2) ? "nd" :
                                            (i+1 == 3) ? "rd" : "th";
                    telemetry.addData((i+1)+suffix+" Order Info", orderInfo);
                }
            } else { // If queue is empty
                telemetry.addData("Queue Status", "Empty");
            }
            telemetry.update();
        }
        reader.shutdown();
        clearGoshDarnit();
    }

    /**
     * Builds a schedule of toppings based on the read input.
     */
    public void processSelection(int selection) {
        List<Integer> newSchedule = new ArrayList<>();
        float newCost = 0.00f;
        String newFlavor = "No Ice Cream";

        // Select flavors
        for (int i = 0; i < NUM_OF_FLAVORS; i++) {
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

            for (int i = NUM_OF_FLAVORS; i < NUM_OF_FLAVORS + NUM_OF_TOPPINGS; i++) {
                if (((selection >> i) & 0x01) == 1) {
                    newSchedule.add(BUTTON_TO_TOPPING_NUM[i - NUM_OF_FLAVORS]);
                }
            }
            Collections.sort(newSchedule);
            scheduleQueue.add(newSchedule);

        // First topping is free, only if with ice cream
        newCost += 0.50f * newSchedule.size(); // Each is 50 cents
        if (!newFlavor.equals("No Ice Cream")) {
            newCost += 2.00f; // Bowl cost
            if (!newSchedule.isEmpty()) { newCost -= 0.50f; }
            // Only apply discount if you have ordered ice cream and at least 1 topping
        }
        costQueue.add(newCost); // Save cost to queue

            if (machineState == MachineState.IDLE) {
                // Set start LED to ON
                operatorLedStates[0] = LedState.ON;
            }
    }

    /**
     * Dispenses a topping by calculating the target position.
     *
     * @param topping the topping index (1-indexed)
     */
    private void dispenseTopping(int topping) {
        if (topping != SERVO_INDEX) { // If not whipped cream
            dispenserTally[topping] += SECTORS_PER_DISPENSE[topping];
            double ticksPerLoad = 2786.2 / DISPENSER_SECTORS[topping];
            int dispenserTarget = (int) (dispenserTally[topping] * ticksPerLoad);
            allMotors[topping].setTargetPosition(dispenserTarget);
        } else { // Whipped cream
            creamServo.setPosition(CREAM_DISPENSE_ANGLE);
            creamDispenseTimer.reset();
        }
    }

    public void startCycle() {
        if (!scheduleQueue.isEmpty()) {
            machineState = MachineState.TARGET_DISPENSER;
            costQueue.remove(0);
            flavorQueue.remove(0);
            currentSchedule = scheduleQueue.remove(0);
        }
        if (isEmergencyStopped) { // Only reset powers if emergency stopped
            for (int i = 0; i < allMotors.length; i++) {
                if (i != SERVO_INDEX) {
                    allMotors[i].setPower(DISPENSER_POWER);
                }
            }
            conveyorMotor.setPower(1);
            isEmergencyStopped = false;
        }
        // Set abort LED to ON
        operatorLedStates[1] = LedState.ON;
        conveyorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        conveyorMotor.setPower(1);
    }

    public void emergencyStop() {
        for (DcMotorEx motor : allMotors) {
            motor.setPower(0); // Kill power to all motors
        }
        creamServo.setPosition(0);
        isEmergencyStopped = true; // Flag for emergency stop
        // Set abort LED to BLINK
        operatorLedStates[1] = LedState.BLINK;
    }

    public void resetSystem() {
        if (!scheduleQueue.isEmpty()) {
            // Set start LED to ON
            operatorLedStates[0] = LedState.ON;
        } else {
            // Set start LED to OFF
            operatorLedStates[0] = LedState.OFF;
        }
        machineState = MachineState.IDLE;
        if (isEmergencyStopped) { // Only reset powers if emergency stopped
            for (int i = 0; i < allMotors.length; i++) {
                if (i != SERVO_INDEX) {
                    allMotors[i].setPower(DISPENSER_POWER);
                }
            }
            conveyorMotor.setPower(1);
            isEmergencyStopped = false;
        }
        // Set abort LED to ON
        operatorLedStates[1] = LedState.ON;
        conveyorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        conveyorMotor.setPower(-1);
    }

    public void confirmSelection(int selection) {
        processSelection(selection);
    }
    public void operatorAction(int button) {
        switch (button) {
            case 0: // Start header
                startCycle();
                break;
            case 1: // Abort header
                emergencyStop();
                break;
            case 2: // Reset header
                resetSystem();
                break;
            }
        }

    public void clearGoshDarnit() {
        for (int i = 0; i < 2; i++) {
            telemetry.clearAll();
        }
    }
}

