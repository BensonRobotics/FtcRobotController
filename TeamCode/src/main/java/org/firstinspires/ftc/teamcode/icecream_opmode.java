package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@TeleOp(name="icecream test2")
public class icecream_opmode extends LinearOpMode {
    private double counts_per_rev = 1440 / (2 * Math.PI * 34.4);

    private ArduinoInterface arduino = new ArduinoInterface();
    private MotorController motorController = new MotorController();
    private ButtonManager buttonManager = new ButtonManager();
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        telemetry.addData("init", "init");
        telemetry.update();
        List<Integer> arduinoState;
        arduino.init(hardwareMap);
        buttonManager.init(hardwareMap, telemetry);
        motorController.init(hardwareMap, buttonManager.estop, runtime, telemetry);

        waitForStart();
        telemetry.addData("start", "start");
        telemetry.update();

        while (opModeIsActive()) {
            // arduino.waitUntilReady();
            buttonManager.telemetryUpdate();

            sleep(1000);
            telemetry.addData("aoeu", "aoeu");
            telemetry.update();

            motorController.belt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorController.belt.setPower(-1);
            buttonManager.waitForStartLimit();
            motorController.belt.setPower(0);
            motorController.belt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorController.belt.setTargetPosition(0);
            motorController.belt.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            buttonManager.waitForStart();
            arduinoState = arduino.getState();

            StringBuilder stringBuilder = new StringBuilder();
            for (int i = 0; i < arduinoState.toArray().length; i++) {
                stringBuilder.append(arduinoState.toArray()[i]);
                if (i < arduinoState.toArray().length - 1) {
                    stringBuilder.append(", ");
                }
            }
            telemetry.addData("Array State", stringBuilder.toString());
            telemetry.update();

            sleep(500);

            for (int i = 0; i < arduinoState.toArray().length; i++) {
                telemetry.addData("test", "%d", i);//arduinoState.toArray()[i]);
                telemetry.update();
                if (motorController.runToTopping(arduinoState.get(i)) == -1) {
                    requestOpModeStop();
                }
                if (arduinoState.get(i) < 6) {
                    if (motorController.dispenseTopping(arduinoState.get(i)) == -1) {
                        requestOpModeStop();
                    }
                }
                if (arduinoState.get(i) == 6) {
                    motorController.dispenseCream();
                }
                telemetry.addData("debug", "%d, %d, %d", i, motorController.belt.getCurrentPosition(), motorController.belt.getTargetPosition());
                telemetry.update();
                sleep(2000);
            }

            motorController.runToPosition(motorController.belt.getCurrentPosition()+30000);

            buttonManager.waitForEndLimit();
            motorController.belt.setPower(0);
            telemetry.addData("end", "end");
            telemetry.update();
            buttonManager.waitForReset();

            while (!buttonManager.estop.getState()) {
                requestOpModeStop();
            }
        }
    }
}
