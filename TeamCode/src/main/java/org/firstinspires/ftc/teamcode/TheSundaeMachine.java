package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Sundae Machine", group = "Off-Season")
public class TheSundaeMachine extends LinearOpMode {

    CustomI2cDriver arduino;
    I2cDeviceSynch i2cDeviceSynch;
    byte[] readCache;
    ElapsedTime timer;

    //Number of digital pins
    int numDigitalInputs = 5;
    //Total size of the buffer
    int bufferSize = numDigitalInputs;
    int outputBufferSize = 1;

    //Pin states
    boolean pin2State;
    boolean pin3State;
    boolean pin4State;
    boolean pin5State;
    boolean pin6State;
    //Output state
    boolean outputState;

    @Override
    public void runOpMode() throws InterruptedException {
        // Get the I2C device
        arduino = hardwareMap.get(CustomI2cDriver.class, "arduino");
        if(arduino == null){
            telemetry.addData("Error", "Arduino not found");
            telemetry.update();
            sleep(5000);
            return;
        }

        i2cDeviceSynch = arduino.getDeviceClient();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive()) {

            //Read from the input register.
            readCache = i2cDeviceSynch.read(CustomI2cDriver.PIN_STATE_INPUT_REGISTER, bufferSize);

            // Get individual pin states
            pin2State = readCache[0] == 1;
            pin3State = readCache[1] == 1;
            pin4State = readCache[2] == 1;
            pin5State = readCache[3] == 1;
            pin6State = readCache[4] == 1;

            //Set the output state
            if (timer.seconds() > 1){
                outputState = !outputState;
                timer.reset();
            }

            //Send the output pin state
            i2cDeviceSynch.write(CustomI2cDriver.PIN_STATE_OUTPUT_REGISTER, new byte[]{outputState ? (byte) 1 : (byte) 0});

            // Display the states
            telemetry.addData("Pin D2", pin2State);
            telemetry.addData("Pin D3", pin3State);
            telemetry.addData("Pin D4", pin4State);
            telemetry.addData("Pin D5", pin5State);
            telemetry.addData("Pin D6", pin6State);
            telemetry.addData("Output State", outputState);
            telemetry.update();
        }
    }
}