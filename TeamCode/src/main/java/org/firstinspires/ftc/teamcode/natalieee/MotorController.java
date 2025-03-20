package org.firstinspires.ftc.teamcode.natalieee;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MotorController {
    private double TOPPING_CLICKS_PER_REV = 1440/(Math.PI*2)*50;
    private double BELT_CPM_PER_REV = 1440/(Math.PI*2*34.4);

    private int[] toppingPositions = {270, 555, 800, 1025, 1275, 1510, 1770};
    private int[] toppingCoefs = {15, 3, 1, 1, 1, 1};
    private DigitalChannel estop;

    private DcMotor[] toppingMotors = new DcMotor[6];
    public DcMotor belt;

    private ElapsedTime runtime;
    private Telemetry telemetry;
    private Servo cream;

    public void init(HardwareMap hardwareMap, DigitalChannel estopButton, ElapsedTime runtimeInstance, Telemetry t) {
        toppingMotors[0] = hardwareMap.get(DcMotor.class, "Topping1");
        toppingMotors[1] = hardwareMap.get(DcMotor.class, "Topping2");
        toppingMotors[2] = hardwareMap.get(DcMotor.class, "Topping3");
        toppingMotors[3] = hardwareMap.get(DcMotor.class, "Topping4");
        toppingMotors[4] = hardwareMap.get(DcMotor.class, "Topping5");
        toppingMotors[5] = hardwareMap.get(DcMotor.class, "Topping6");

        cream = hardwareMap.get(Servo.class, "cream");

        estop = estopButton;

        runtime = runtimeInstance;
        telemetry = t;

        belt = hardwareMap.get(DcMotor.class, "belt");
        belt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        belt.setTargetPosition(0);
        belt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        for (DcMotor motor : toppingMotors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setDirection(DcMotor.Direction.FORWARD);
        }
    }

    public int runToTopping(int topping) {
        if (!estop.getState()) return -1;
        belt.setPower(0.5);
        belt.setTargetPosition((int) Math.round(toppingPositions[topping]*BELT_CPM_PER_REV));

        telemetry.addData("motor", "target: %d", (int) Math.round(toppingPositions[topping]*BELT_CPM_PER_REV));

        while (Math.round(belt.getCurrentPosition()/10) != Math.round(belt.getTargetPosition()/10) && estop.getState()) {
            telemetry.addData("motor", "target: %d", (int) Math.round(toppingPositions[topping]*BELT_CPM_PER_REV));
            telemetry.addData("motor", "current: %d", belt.getCurrentPosition());
            telemetry.update();
        }
        belt.setPower(0);

        return 0;
    }

    public void runToPosition(int position) {
        belt.setPower(1 );
        belt.setTargetPosition(position);
    }

    public void dispenseCream() {
        double stop1 = runtime.seconds() + 1;
        double stop2 = runtime.seconds() + 1.75;
        cream.setPosition(0);
        while (runtime.seconds() <= stop1) {}
        cream.setPosition(0.75);
        while (runtime.seconds() <= stop2) {}
        cream.setPosition(0);
    }

    public int dispenseTopping(int topping) {
        if (!estop.getState()) return -1;

        if (topping == 7) {topping = 2;}

        telemetry.addData("topping", "topping: %d", topping);

        if (topping >= 2) {
            telemetry.addData("first case", "first case");
            toppingMotors[topping].setTargetPosition(toppingMotors[topping].getCurrentPosition() + (int) TOPPING_CLICKS_PER_REV * toppingCoefs[topping]);
            toppingMotors[topping].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            toppingMotors[topping].setPower(1);

            while ((toppingMotors[topping].getCurrentPosition() < toppingMotors[topping].getTargetPosition()) && estop.getState()) {}
            toppingMotors[topping].setPower(0);
        }
        else if (topping <= 1) {
            toppingMotors[topping].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("second case", "second case");
            double stopTime = runtime.seconds() + toppingCoefs[topping];
            toppingMotors[topping].setPower(1);
            while (runtime.seconds() <= stopTime) {}
            toppingMotors[topping].setPower(0);
        }
        else {}
        telemetry.update();

        return 0;
    }
}
