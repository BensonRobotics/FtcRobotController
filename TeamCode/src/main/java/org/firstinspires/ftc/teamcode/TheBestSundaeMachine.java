package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.content.Context;
import android.hardware.usb.UsbManager;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;


import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

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
    private double lastCreamTime = 0;

    // Constants
    private static final Locale LOCALE = Locale.US;
    private static final int NUM_OF_TOPPINGS = 7;
    private static final int NUM_OF_FLAVORS = 3;
    private static final int TOPPING_FALL_WAIT = 500; // Half second delay
    private static final double DISPENSER_POWER = 0.5;
    private static final int CONVEYOR_CURRENT_LIMIT = 5000;

    // Positions and loads (placeholders, update as needed)
    // Make sure these are all the same length as NUM_OF_TOPPINGS
    private final int[] BUTTON_TO_TOPPING_NUM = {0, 1, 2, 3, 4, 5, 6};
    private final int[] OSCILLATION_AMP = {0, 0, 0, 0, 0, 0, 0}; // in ticks
    private final int[] OSCILLATION_FREQ = {1, 1, 1, 1, 1, 1, 1};
    private final int[] BOWL_POSITIONS = {1977, 3615, 5049, 6667, 8313, 9930, 11857};
    private final int[] DISPENSER_SECTORS = {8, 8, 8, 8, 8, 8, 0}; // -1 is servo, invalid
    private final int[] SECTORS_PER_DISPENSE = {2, 2, 2, 2, 2, 2, 0}; // Same
    private final int CREAM_DISPENSE_DURATION = 1000;
    private final float CREAM_DISPENSE_ANGLE = 0.175f;
    private int[] dispenserTally = new int[NUM_OF_TOPPINGS];

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
    // LED states are reversed, so false is on and true is off; digital i/o used as sink

    // Topping schedule and current topping
    private List<List<Integer>> scheduleQueue = new ArrayList<>();
    private List<String> flavorQueue = new ArrayList<>();
    private List<Float> costQueue = new ArrayList<>();
    private List<Integer> currentSchedule = new ArrayList<>();
    private int currentTopping = 0;
    private boolean isEmergencyStopped = false;
    private MachineState lastMachineState = MachineState.IDLE;
    private boolean[] lastButtonStates = new boolean[operatorButtons.length];
    // All will be set to true in setup
    private int lastQueueLength = 0;
    private double lastConveyorPower = 0;
    private boolean isConveyorCurrentTripped = false;
    private LedState[] operatorLedStates = new LedState[] {
            LedState.OFF,
            LedState.ON,
            LedState.ON
    };
    private ElapsedTime[] ledBlinkTimers = new ElapsedTime[operatorLeds.length];
    private ElapsedTime debounceTimer = new ElapsedTime();
    private ElapsedTime bowlDepositTimer = new ElapsedTime();

    // State management using enum
    private enum MachineState {
        IDLE,
        TARGETING_DISPENSER,
        TRAVELLING,
        DISPENSING,
        WAITING_FOR_TOPPING_FALL,
        FINISHING,
        DEPOSITING,
        RESETTING,
        EMERGENCY
    }
    private MachineState machineState = MachineState.IDLE;
    private enum LedState { ON, OFF, BLINK }

    UsbSerialReader reader = new UsbSerialReader();

    @Override
    public void runOpMode() throws InterruptedException {
        Arrays.fill(dispenserTally, 0);
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
        conveyorMotor.setCurrentAlert(CONVEYOR_CURRENT_LIMIT, CurrentUnit.MILLIAMPS);

        for (int i = 0; i < operatorButtons.length; i++) {
            operatorButtons[i] = hardwareMap.get(DigitalChannel.class, operatorButtonNames[i]);
            operatorButtons[i].setMode(DigitalChannel.Mode.INPUT);
        }
        for (int i = 0; i < operatorLeds.length; i++) {
            operatorLeds[i] = hardwareMap.get(DigitalChannel.class, operatorLedNames[i]);
            operatorLeds[i].setMode(DigitalChannel.Mode.OUTPUT);
            operatorLeds[i].setState(true); // Off
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
        bowlDepositTimer.reset();
        for (ElapsedTime timer : ledBlinkTimers) {
            timer.reset();
        }

        processSelection(993);

        while (opModeIsActive()) {

            // Check operator buttons and act ONCE if they are pressed and debounced
            // Remember that button presses are falling edge
            for (int i = 0; i < operatorButtons.length; i++) {
                if (!operatorButtons[i].getState()) {
                    if (!lastButtonStates[i] && debounceTimer.milliseconds() > 100) { // If first time pressed since last
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
                    case ON: operatorLeds[i].setState(false);
                    break;
                    case OFF: operatorLeds[i].setState(true);
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
                case TARGETING_DISPENSER:
                    if (!currentSchedule.isEmpty()) {
                        currentTopping = currentSchedule.remove(0);
                        conveyorMotor.setTargetPosition(BOWL_POSITIONS[currentTopping]);
                        machineState = MachineState.TRAVELLING;
                    } else {
                        machineState = MachineState.FINISHING;
                        conveyorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        conveyorMotor.setPower(1);
                    }
                    break;
                case TRAVELLING:
                    if (!conveyorMotor.isBusy()) {
                        dispenseTopping(currentTopping);
                        machineState = MachineState.DISPENSING;
                    }
                    break;
                case DISPENSING:
                    if (OSCILLATION_AMP[currentTopping] != 0) {
                        conveyorMotor.setTargetPosition(BOWL_POSITIONS[currentTopping] +
                                oscillatedOffset(currentTopping));
                    }

                    if (currentTopping != SERVO_INDEX) {
                        if (!allMotors[currentTopping].isBusy()) {
                            toppingFallTimer.reset();
                            machineState = MachineState.WAITING_FOR_TOPPING_FALL;
                            conveyorMotor.setTargetPosition(BOWL_POSITIONS[currentTopping]);
                        }
                    } else {
                        if (creamDispenseTimer.milliseconds() + lastCreamTime >
                                CREAM_DISPENSE_DURATION) {
                            creamServo.setPosition(0);
                            toppingFallTimer.reset();
                            machineState = MachineState.WAITING_FOR_TOPPING_FALL;
                            conveyorMotor.setTargetPosition(BOWL_POSITIONS[currentTopping]);
                        }
                    }
                    break;
                case WAITING_FOR_TOPPING_FALL:
                    if (toppingFallTimer.milliseconds() > TOPPING_FALL_WAIT) {
                        // Done dispensing
                        machineState = MachineState.TARGETING_DISPENSER;
                    }
                    break;
                case FINISHING:
                    if (!maxEndstop.getState()) {
                        lastConveyorPower = conveyorMotor.getPower();
                        conveyorMotor.setPower(0);
                        machineState = MachineState.DEPOSITING;
                        bowlDepositTimer.reset();
                    }
                    break;
                case DEPOSITING:
                    if (bowlDepositTimer.milliseconds() > 1000) {
                        resetSystem();
                    }
                    break;
                case RESETTING:
                    if (!minEndstop.getState()) {
                        lastConveyorPower = conveyorMotor.getPower();
                        conveyorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        operatorLedStates[2] = LedState.ON;
                        machineState = MachineState.IDLE;
                    }
                    break;
            }

            /*if (conveyorMotor.isOverCurrent()) {
                lastConveyorPower = conveyorMotor.getPower();
                conveyorMotor.setPower(0);
                isConveyorCurrentTripped = true;
                conveyorMotor.setCurrentAlert(CONVEYOR_CURRENT_LIMIT/2.0, CurrentUnit.MILLIAMPS);
            } else if (isConveyorCurrentTripped) {
                conveyorMotor.setPower(lastConveyorPower);
                isConveyorCurrentTripped = false;
                conveyorMotor.setCurrentAlert(CONVEYOR_CURRENT_LIMIT, CurrentUnit.MILLIAMPS);
            } */

            if (lastQueueLength != scheduleQueue.size()) {
                clearGoshDarnit();
                lastQueueLength = scheduleQueue.size();
            }
            if (!scheduleQueue.isEmpty()) { // If there is something in one of the cues
                for (int i = 0; i < scheduleQueue.size(); i++) {
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

    private void dispenseTopping(int topping) {
        if (topping != SERVO_INDEX) { // If not whipped cream
            dispenserTally[topping] += SECTORS_PER_DISPENSE[topping];
            // Assuming 30 rpm motor, 5281.1 ticks per revolution
            // If using different motor, please change this to the listed ticks per revolution
            double ticksPerLoad = 5281.1 / DISPENSER_SECTORS[topping];
            int dispenserTarget = (int) (dispenserTally[topping] * ticksPerLoad);
            allMotors[topping].setTargetPosition(dispenserTarget);
        } else { // Whipped cream
            creamServo.setPosition(CREAM_DISPENSE_ANGLE);
            creamDispenseTimer.reset();
            lastCreamTime = 0;
        }
    }

    private double dispenseCompletion(int index) {
        if (index != SERVO_INDEX) {
            double ticksPerLoad = 5281.1 / DISPENSER_SECTORS[index];
            double dispenserTarget = dispenserTally[index] * ticksPerLoad;
            double oldTarget = (dispenserTally[index] - SECTORS_PER_DISPENSE[index]) * ticksPerLoad;
            double range = dispenserTarget - oldTarget;
            double thoseWhoKnow = allMotors[index].getCurrentPosition() - oldTarget;
            return thoseWhoKnow / range;
        } else {
            return (creamDispenseTimer.milliseconds() + lastCreamTime) / CREAM_DISPENSE_DURATION;
        }
    }

    private int oscillatedOffset(int index) {
        double offset = Math.sin(2*Math.PI * dispenseCompletion(index) *
                OSCILLATION_FREQ[index]) * OSCILLATION_AMP[index];
        return (int) offset;
    }

    public void startCycle() {
        // startCycle no longer resets emergency mode
        if (!scheduleQueue.isEmpty()) {
            machineState = MachineState.TARGETING_DISPENSER;
            costQueue.remove(0);
            flavorQueue.remove(0);
            currentSchedule = scheduleQueue.remove(0);
            operatorLedStates[0] = LedState.BLINK;
            conveyorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            conveyorMotor.setPower(1);
        }
    }

    public void emergencyStop() {
        for (DcMotorEx motor : allMotors) {
            if (motor != null) {
                motor.setPower(0); // Kill power to all motors
            }
        }
        creamServo.setPosition(0.01);
        lastConveyorPower = conveyorMotor.getPower();
        lastCreamTime = creamDispenseTimer.milliseconds();
        lastMachineState = machineState;
        machineState = MachineState.EMERGENCY;
        // Set abort LED to BLINK
        operatorLedStates[1] = LedState.BLINK;
    }

    public void resetSystem() {
        if (machineState == MachineState.EMERGENCY) { // Only reset powers if emergency stopped
            for (DcMotorEx allMotor : allMotors) {
                if (allMotor != null) {
                    allMotor.setPower(DISPENSER_POWER);
                }
            }
            operatorLedStates[1] = LedState.ON;
            conveyorMotor.setPower(lastConveyorPower);
            creamDispenseTimer.reset();
            machineState = lastMachineState;
            if (creamServo.getPosition() == 0.01) {
                creamServo.setPosition(CREAM_DISPENSE_ANGLE);
            }
        } else {
            operatorLedStates[2] = LedState.BLINK;
            if (!scheduleQueue.isEmpty()) {
                // Set start LED to ON
                operatorLedStates[0] = LedState.ON;
            } else {
                // Set start LED to OFF
                operatorLedStates[0] = LedState.OFF;
            }
            machineState = MachineState.RESETTING;
            // Set abort LED to ON
            conveyorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            conveyorMotor.setPower(-1);
        }
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
        for (int i = 0; i < 3; i++) {
            telemetry.clearAll();
        }
    }
}

