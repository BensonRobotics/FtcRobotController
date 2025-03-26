package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
/**
 * THIS WHOLE THING IS USELESS
 * PURELY FOR EXHIBITORY REASONS
 * SORRY FOR THE SWEARING
 **/
@Disabled
@TeleOp(name = "Sundae Machine", group = "Off-Season")
public class TheOldSundaeMachine extends LinearOpMode {

    // CustomI2cDriver arduino;
    I2cDeviceSynch i2cDeviceSynch;
    byte[] readCache;
    ElapsedTime timer;
    ElapsedTime gravityTimer;
    ElapsedTime reEnableButtonsTimer;

    //Number of digital pins
    int numDigitalInputs = 5;
    //Total size of the buffer
    int bufferSize = numDigitalInputs;
    boolean outputState;
    int machineState = 0; // Waiting
    int currentTopping = -1; // The -1th topping is Kellog's Black Hole Bits
    ArrayList<Integer> toppingSchedule = new ArrayList<>();
    final int endPosition = 10000; // PLACEHOLDER, change this
    final int[] bowlPositions = {1000, 2000, 3000, 4000, 5000}; // PLACEHOLDERS, change these
    final int[] dispenserLoadsPerRotation = {8, 8, 8, 8, 8};
    final int[] loadsPerDispense = {1, 1, 1, 1, 1}; // PLACEHOLDERS, change these
    int[] dispenseCount = {0, 0, 0, 0, 0};

    DcMotorEx conveyorMotor;
    DcMotorEx topping1Motor;
    DcMotorEx topping2Motor;
    DcMotorEx topping3Motor;
    DcMotorEx topping4Motor;
    DcMotorEx topping5Motor;
    DcMotorEx[] allMotors = {conveyorMotor, topping1Motor, topping2Motor, topping3Motor, topping4Motor, topping5Motor};
    String[] allMotorNames = {"conveyorMotor", "topping1Motor", "topping2Motor", "topping3Motor", "topping4Motor", "topping5Motor"};

    @Override
    public void runOpMode() throws InterruptedException {
        // Get the I2C device
        // arduino = hardwareMap.get(CustomI2cDriver.class, "arduino");
        // i2cDeviceSynch = arduino.getDeviceClient();

        // For each motor, assign hardwareMap and set all parameters
        // I am so fucking pissed that you can't just get the object name as a string
        // Like allMotors[i].toString gives some giant hash code, I am fricking livid
        // And .getDeviceName just gives some captain america-ass "I am a REV HD Hex Motor! :)"
        for (int i = 0; i < allMotors.length; i++) {
            allMotors[i] = hardwareMap.get(DcMotorEx.class, allMotorNames[i]);
            allMotors[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            allMotors[i].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            allMotors[i].setTargetPosition(0);
            allMotors[i].setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            allMotors[i].setPower(0.5);
        }
        conveyorMotor.setPower(1); // Upgrade the conveyor

        outputState = true; // Enable button relay at start

        telemetry.addData("Status", "Initialized"); // We made it this far...
        telemetry.update();

        waitForStart();

        // Timer hell, kill me now
        timer = new ElapsedTime();
        gravityTimer = new ElapsedTime();
        reEnableButtonsTimer = new ElapsedTime();
        timer.reset();
        gravityTimer.reset();
        reEnableButtonsTimer.reset();
        while (opModeIsActive()) { // Loop the loop

            //Read from the input register.
            // readCache = i2cDeviceSynch.read(CustomI2cDriver.PIN_STATE_INPUT_REGISTER, bufferSize);

            // Top, bottom, or switch-case?
            switch (machineState) {
                case 0: // Waiting for order start
                    if (readCache[0] == 1) { // if start button pressed
                        for (int i = 1; i < numDigitalInputs; i++) { // Build topping list
                            // Start indexing at 1 since pin2 is the start button
                            if (readCache[i] == 1) { // Only add if topping is selected
                                toppingSchedule.add(i);
                            }
                            // We now have a list of topping indexes!
                            // For example, {1,2,5}

                            // Oh my naive optimism, as seen above
                        }
                        outputState = false; // Pulse off relay to control panel
                        reEnableButtonsTimer.reset();
                        machineState = 1;
                    }
                    break;
                case 1: // Advance to next topping
                    if (!toppingSchedule.isEmpty()) {
                        currentTopping = toppingSchedule.get(0); // Jot down next topping
                        toppingSchedule.remove(0); // Remove it from the list
                        conveyorMotor.setTargetPosition(bowlPositions[currentTopping - 1]);
                        machineState = 2;
                    } else {
                        machineState = 5; // We're done! AAAAAAAAAARRRGGGGHHHHHH
                    }
                    break;
                case 2: // Once at dispenser, dispense that topping
                    if (!conveyorMotor.isBusy()) {
                        dispenseTopping(currentTopping);
                        machineState = 3;
                    }
                    break;
                case 3: // Wait for dispenser to finish, ha ha
                    if (!allMotors[currentTopping].isBusy()) {
                        gravityTimer.reset(); // Wait for toppings to fall
                        machineState = 4;
                    }
                    break;
                case 4: // Wait for gravity, this is bullshit, fuck you Isaac Newton
                    // Come to think of it, I should make a time constant that is calculated
                    // from the acceleration of gravity, air resistance of an eminem, and the
                    // distance it has to fall. Maybe then will I achieve peak optimization.
                    if (gravityTimer.seconds() > 1) {
                        machineState = 1;
                    }
                    break;
                case 5:
                    conveyorMotor.setTargetPosition(endPosition);
                    // Finally, inner peace. Unless some dickhead forgot to press the Oreos button
                    // and is being stubborn.
                    // How much of my code will be comments by the time I'm done with my
                    // sleep-deprived rambling? And I know, it's quite the overplayed trope.
                    // This reads like a greentext story, killing myself instantly
            }

            // Wake up the buttons! Who the fuck needs rest?? Not the buttons!!!!
            if (reEnableButtonsTimer.milliseconds() > 100){
                outputState = true;
            }
            //Send the output pin state
            // i2cDeviceSynch.write(CustomI2cDriver.PIN_STATE_OUTPUT_REGISTER,
                    // new byte[]{outputState ? (byte) 1 : (byte) 0});
        }
    }

    // This whole thing reminds me of Thneedville because uhh they have air in bottles
    public void dispenseTopping(int topping) {
        dispenseCount[topping - 1]++;
        double ticksPerLoad = (2786.2 / dispenserLoadsPerRotation[topping - 1]);
        int dispenserTarget = (int) (dispenseCount[topping - 1] *
                loadsPerDispense[topping - 1] * ticksPerLoad);
        allMotors[topping].setTargetPosition(dispenserTarget);
    }
}
// Sorry for all the swearing, my mind is melting like the faces on the clock.
// Alright, goodnight, time to force chatgpt to optimize my code so I don't look like a psychopath.