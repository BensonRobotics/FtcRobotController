package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;



public class ButtonManager {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private DigitalChannel limit1;
    private DigitalChannel limit2;
    private DigitalChannel start;
    private DigitalChannel reset;
    public DigitalChannel estop;

    public void init(HardwareMap hardwareMap, Telemetry t) {
        limit1 = hardwareMap.get(DigitalChannel.class, "limit1");
        limit2 = hardwareMap.get(DigitalChannel.class, "limit2");
        start = hardwareMap.get(DigitalChannel.class, "start");
        reset = hardwareMap.get(DigitalChannel.class, "reset");
        estop = hardwareMap.get(DigitalChannel.class, "estop");
        telemetry = t;
    }

    public void telemetryUpdate() {
        telemetry.addData("limits", "limit1: %b", limit1.getState());
        telemetry.addData("limits", "limit2: %b", limit2.getState());
        telemetry.update();
    }

    public void waitForEndLimit() {
        while (limit1.getState() && estop.getState()) {}

        return;
    }

    public void waitForStartLimit() {
        while (limit2.getState() && estop.getState()) {}

        return;
    }

    public void waitForReset() {
        while (reset.getState() && estop.getState()) {}

        return;
    }

    public void waitForStart() {
        while (start.getState()) {}
        return;
    }
}
