package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

@TeleOp(name="TeleOp", group="Linear Opmode")
public class TeleOpV1 extends LinearOpMode {
    
    private boolean[] button = {false, false, false, false, false};
    // 0 = compass, 1 = lift wheels, 2 = duck left, 3 = duck right, 4 = collector toggle
    private boolean[] toggle = {true, true, false, false, false};
    // 0 = compass, 1 = lift wheels, 2 = duck left, 3 = duck right, 4 = collector toggle
    
    boolean calibrated = false;
    int liftPos = 0;
    int liftZero = 0;
    int liftStart = 0;
    final int LIFT_MAX = 380;
    final int LIFT_MIN = -5;
    
    RobotHardware H = new RobotHardware();
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        ////////////////////////////// Init //////////////////////////////
        
        MecanumWheelDriverV2 drive = new MecanumWheelDriverV2(H);
        ElapsedTime runtime = new ElapsedTime();
        ExecutorService pool = Executors.newFixedThreadPool(2);
        H.init(hardwareMap, this);
        pool.execute(H);
        H.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ////////////////////////////// Init Variables //////////////////////////////
        
        
        double y;
        double x;
        double rotate = 0;
        double rotateScaled = 0;
        double radius = 0;
        double power = 0;
        double headingLock = H.heading;
        boolean allowStartHeadingLock = true;
        double angle = 0;
        double rotateRadius;
        boolean barrierMode = false;
        boolean traversingForwards = true;
        int traversalStep = 0;
        long lastTurnTime = 0;
        
        double upPower = 0;
        double downPower = 0;
        
        double agl_frwd = 0;
        double heading = 0;
        
        waitForStart();
        runtime.reset();
        
        while (opModeIsActive()) {
    
            ////////////////////////////// Set Variables //////////////////////////////
    
            y = gamepad1.left_stick_y;
            x = -gamepad1.left_stick_x;
    
            rotate += powerFollow(rotate, exponentialScaling(Range.clip(gamepad1.right_stick_x, -1, 1)));
    
            radius = exponentialScaling(Range.clip(Math.hypot(x, y), 0, 1));
            power += powerFollow(power, radius);
            if (radius > 0.05) angle = Math.toDegrees(Math.atan2(y, x)) + 90 + agl_frwd - heading;
            //drive.StrafePowerMove(angle, power*0.6 + (gamepad1.right_trigger*0.4), 1);
            //rotateScaled = rotate * 0.6 + gamepad1.right_trigger*0.4;
            drive.StrafePowerMove(angle, power*0.75, 1);
            rotateScaled = rotate * 0.75;
            
            if (Math.abs(rotate) < 0.05) {
                if (lastTurnTime + 350 < runtime.now(TimeUnit.MILLISECONDS)) {
                    if (Math.abs(drive.findDegOffset(H.heading, headingLock)) > 1.5) {
                        if (allowStartHeadingLock) {
                            drive.HeadingRotate(headingLock, 0.15, 1);
                            allowStartHeadingLock = false;
                        }
                    } else {
                        allowStartHeadingLock = true;
                    }
                } else {
                    headingLock = H.heading;
                }
            } else {
                lastTurnTime = runtime.now(TimeUnit.MILLISECONDS);
                headingLock = H.heading;
                drive.PowerRotate(rotateScaled, 1);
            }
    
            drive.startActions();
            
            upPower = Range.clip(-gamepad2.left_stick_y, 0, 1);
            downPower += Range.clip(gamepad2.left_stick_y, 0, 1);
            
            setLiftPower(upPower, downPower);
            downPower = 0;
            
            setRampPower(-gamepad2.right_stick_y);
            
            ////////////////////////////// Buttons //////////////////////////////
            
            toggleButton(gamepad1.back, 0); // compass
            
            if (toggle[0]) {
                
                heading = H.heading;
                
            } else {
                
                heading = agl_frwd;
                
            }
            
            toggleButton(gamepad1.a || gamepad2.a, 1);
            toggleButton(gamepad1.x, 2);
            toggleButton(gamepad1.b, 3);
            toggleButton(gamepad1.right_bumper, 4);
            
            if (toggle[1]) {
                H.wheelLift[0].setPosition(0);
                H.wheelLift[1].setPosition(1);
                H.wheelLift[2].setPosition(1);
                H.wheelLift[3].setPosition(0);
            } else {
                H.wheelLift[0].setPosition(1);
                H.wheelLift[1].setPosition(0);
                H.wheelLift[2].setPosition(0);
                H.wheelLift[3].setPosition(1);
            }
            
            H.collectorMotor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            
            if (toggle[2]) {
                toggle[3] = false;
                H.duckServo.setPosition(0);
            }
            
            if (toggle[3]) {
                H.duckServo.setPosition(1);
            }
            
            if (!toggle[2] && !toggle[3]) {
                H.duckServo.setPosition(0.5);
            }
            
            if (toggle[4]) {
                H.collectorServo.setPosition(1);
            } else {
                H.collectorServo.setPosition(0);
            }
            
            if (gamepad1.start) {
                agl_frwd = heading;
            }
            
            //telemetry.addData("wheelPower: ", H.driveMotor[0].getPower());
            //telemetry.addData("wheelPower: ", H.driveMotor[1].getPower());
            //telemetry.addData("wheelPower: ", H.driveMotor[2].getPower());
            telemetry.addData("lift Power: ", H.liftMotor.getPower());
            telemetry.addData("R_range", H.rightDistance);
            telemetry.addData("L_range", H.leftDistance);
            telemetry.addData("F_range", H.frontDistance);
            //telemetry.addData("max wheel power", drive.maxWheelPower);
            //telemetry.addData("barrier mode: ", barrierMode);
            //telemetry.addData("direction: ", traversingForwards);
            //telemetry.addData("heading", H.heading);
            //telemetry.addData("heading lock", drive.findDegOffset(H.heading, headingLock));
            telemetry.update();
            
        }
        
        pool.shutdownNow();
        
    }
    
    private void setLiftPower(double upPower, double downPower) {
        
        liftPos = H.liftMotor.getCurrentPosition() - liftZero;
        telemetry.addData("lift pos: ", liftPos);
    
        if (Math.abs(downPower) < 0.05 && Math.abs(upPower) < 0.05) {
            H.liftMotor.setPower(0);
            liftStart = liftPos;
            return;
        }
        if (!H.liftStop.getState()) {
            liftZero = H.liftMotor.getCurrentPosition();
            H.liftMotor.setPower(exponentialScaling(upPower));
            calibrated = true;
            return;
        }
        
        if (calibrated && liftPos >= LIFT_MAX) {
            H.liftMotor.setPower( -exponentialScaling(downPower));
            return;
        }
        
        if (calibrated) {
            H.liftMotor.setPower(adaptivePowerRamping(LIFT_MAX - liftPos, exponentialScaling(upPower), LIFT_MAX - liftStart) - adaptivePowerRamping(LIFT_MIN - liftPos, exponentialScaling(downPower), LIFT_MIN - liftStart));
            return;
        }
        
        H.liftMotor.setPower(exponentialScaling(upPower) - exponentialScaling(downPower));
    }
    
    private void setRampPower(double power) {
        
        power = (power + 1) * 0.5;
        
        if (Math.abs(power - 0.5) < 0.025) {
            H.rampServo.setPosition(0.5);
            return;
        }
        if (!H.rampStop[0].getState()) {
            H.rampServo.setPosition(Range.clip(power,0.5,1));
            return;
        }
        
        if (!H.rampStop[1].getState()) {
            H.rampServo.setPosition(Range.clip(power,0,0.5));
            return;
        }
        
        H.rampServo.setPosition(power);
    }
    
    private void toggleButton(boolean gamepadIn, int numb) {
        
        if (gamepadIn) {
            
            if (!button[numb]) {
                
                toggle[numb] = !toggle[numb];
                button[numb] = true;
                
            }
            
        } else {
            
            button[numb] = false;
            
        }
        
    }
    
    double exponentialScaling(double input) {
        if (Math.abs(input) > H.STICK_DEAD_ZONE) {
            return Math.signum(input) * ((Math.pow(H.EXP_BASE, Math.abs(input)) - 1) * (1 - H.INITIAL_VALUE) / (H.EXP_BASE - 1) + H.INITIAL_VALUE);
        } else {
            return 0;
        }
    }
    
    double powerFollow(double currentPower, double goalPower) {
        if (Math.abs(goalPower - currentPower) >= H.POWER_FOLLOW_INCREMENT) {
            return Math.signum(goalPower - currentPower) * H.POWER_FOLLOW_INCREMENT;
        } else {
            return goalPower - currentPower;
        }
    }
    
    private double adaptivePowerRamping(double offset, double power, double initialOffset) {
        
        return Range.clip((Math.exp(0.1 * Math.abs(offset))-1)/(Math.abs(power) * Math.pow(Math.abs(initialOffset/12),3)), 0.1, Math.abs(power));
        
    }
    
}

/* Unused

class Controller {
 
    
    Gamepad gamepad1;
    Gamepad gamepad2;
    
    boolean twoController;
    
    double move_x;
    double move_y;
    double rotate;
    double freight_lift_up;
    double freight_lift_down;
    double freight_lift;
    double ramp_lift_up;
    double ramp_lift_down;
    double ramp_lift;
    
    Controller(Gamepad gamepad1) {
        this.gamepad1 = gamepad1;
        twoController = false;
        
        move_x = this.gamepad1.left_stick_x;
        move_y = this.gamepad1.left_stick_y;
    }
    
    Controller(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        twoController = true;
        
    }
}*/
