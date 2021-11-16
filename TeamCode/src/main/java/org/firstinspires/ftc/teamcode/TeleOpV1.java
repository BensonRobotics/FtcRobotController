package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@TeleOp(name="TeleOp", group="Linear Opmode")
public class TeleOpV1 extends LinearOpMode {
    
    private boolean[] button = {false, false, false, false, false, false, false};
    // 0 = half speed, 1 = compass, 2 = POV, 3 = front wheels, 4 = back wheels, 5 = duck on/off, 6 = duck switch directions
    private boolean[] toggle = {true, true, false, true, true, false, false};
    // 0 = half speed, 1 = compass, 2 = POV, 3 = front wheels, 4 = back wheels, 5 = duck on/off, 6 = duck switch directions
    
    
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
        ////////////////////////////// Init Variables //////////////////////////////
        
        
        double y;
        double x;
        double rotate;
        double radius = 0;
        double power = 0;
        double headingLock = H.heading;
        boolean allowStartHeadingLock = true;
        double angle = 0;
        double rotateRadius;
        boolean barrierMode = false;
        boolean traversingForwards = true;
        int traversalStep = 0;
        
        double agl_frwd = 0;
        double heading = 0;
        
        waitForStart();
        runtime.reset();
        
        while (opModeIsActive()) {
    
            ////////////////////////////// Set Variables //////////////////////////////
    
            y = gamepad1.left_stick_y;
            x = -gamepad1.left_stick_x;
    
            if (toggle[2]) {
        
                rotateRadius = exponentialScaling(Range.clip(Math.hypot(-gamepad1.right_stick_x, gamepad1.right_stick_y), 0, 1));
        
                if (rotateRadius > 0.2) {
                    //target = drive.addDegree(Math.toDegrees(Math.atan2(gamepad1.right_stick_y, -gamepad1.right_stick_x)), -90);
                    rotate = 1;//drive.POVRotate(target + agl_frwd, rotateRadius);
                } else {
                    rotate = 0;
                }
        
            } else {
        
                rotate = exponentialScaling(Range.clip(gamepad1.right_stick_x, -1, 1));
        
            }
    
            radius = exponentialScaling(Range.clip(Math.hypot(x, y), 0, 1));
            power += powerFollow(power, radius);
            if (radius > 0.05) angle = Math.toDegrees(Math.atan2(y, x)) + 90 + agl_frwd - heading;
    
            if (toggle[0]) {
                drive.StrafePowerMove(angle, power*0.6, 1);
                rotate *= 0.6;
            } else {
                drive.StrafePowerMove(angle, power, 1);
            }
    
            //telemetry.addData("angle", angle);
            //if (allowStartHeadingLock) {
            //}
            
            if (Math.abs(rotate) < 0.05) {
                if (Math.abs(drive.findDegOffset(H.heading, headingLock)) > 2.5) {
                    if (allowStartHeadingLock) {
                        drive.HeadingRotate(headingLock, 0.3, 1);
                        allowStartHeadingLock = false;
                    }
                } else {
                    allowStartHeadingLock = true;
                }
            } else {
                headingLock = H.heading;
                drive.PowerRotate(rotate, 1);
            }
    
            drive.setWheelPower();
    
            /*if (!barrierMode && (H.wheelTriggers[0].getState() || H.wheelTriggers[1].getState() || H.wheelTriggers[2].getState() || H.wheelTriggers[3].getState())) {
                //set up barrier traversal information
                barrierMode = true;
                traversingForwards = (H.wheelTriggers[0].getState() || H.wheelTriggers[1].getState()) && !(H.wheelTriggers[2].getState() || H.wheelTriggers[3].getState());
                traversalStep = 0;
            }
            if (barrierMode) {
                
                if (radius > 0.2 && (traversingForwards==Math.abs(angle) > 90)) {
                    // if diver is reversing change traversing direction;
                    traversingForwards = !traversingForwards;
                }
                
                switch (traversalStep) {
                    case 0: // Lift front wheels
                        if (traversingForwards) {
                            H.wheelLift[0].setPosition(1);
                            H.wheelLift[1].setPosition(1);
                        } else {
                            H.wheelLift[2].setPosition(1);
                            H.wheelLift[3].setPosition(1);
                        }
                        break;
                    case 1: // Swap wheels on ground
                        if (traversingForwards) {
                            H.wheelLift[0].setPosition(0);
                            H.wheelLift[1].setPosition(0);
                            H.wheelLift[2].setPosition(1);
                            H.wheelLift[3].setPosition(1);
                        } else {
                            H.wheelLift[0].setPosition(1);
                            H.wheelLift[1].setPosition(1);
                            H.wheelLift[2].setPosition(0);
                            H.wheelLift[3].setPosition(0);
                        }
                        break;
                    case 2: // Drop back wheels and exit barrier mode
                        if (traversingForwards) {
                            H.wheelLift[2].setPosition(0);
                            H.wheelLift[3].setPosition(0);
                        } else {
                            H.wheelLift[0].setPosition(0);
                            H.wheelLift[1].setPosition(0);
                        }
                        barrierMode = false;
                        break;
                        
                }
                
            }*/
            ////////////////////////////// Buttons //////////////////////////////
            
            toggleButton(gamepad1.left_stick_button, 0); // half speed
            
            toggleButton(gamepad1.back, 1); // compass
            
            if (toggle[1]) {
                
                heading = H.heading;
                
            } else {
                
                heading = agl_frwd;
                
            }
    
            toggleButton(gamepad1.right_stick_button, 2); // POV
            
            toggleButton(gamepad1.a, 3);
            
            toggleButton(gamepad1.y, 4);
            
            if (toggle[3]) {
                H.wheelLift[0].setPosition(0);
                H.wheelLift[1].setPosition(1);
            } else {
                H.wheelLift[0].setPosition(1);
                H.wheelLift[1].setPosition(0);
            }
            
            if (toggle[4]) {
                H.wheelLift[2].setPosition(1);
                H.wheelLift[3].setPosition(0);
            } else {
                H.wheelLift[2].setPosition(0);
                H.wheelLift[3].setPosition(1);
            }
            
            //toggleButton(gamepad1.right_bumper || gamepad1.b, 3); // launcher
            
            //toggleButton(gamepad1.left_trigger > 0.25, 4);
            
            toggleButton(gamepad1.left_bumper, 5);
            
            toggleButton(gamepad1.right_bumper, 6);
            
            if (toggle[5]) {
                if (toggle[6]) {
                    //H.collectorMotor.setPower(-1);
                    H.duckServo.setPosition(0);
                } else {
                    //H.collectorMotor.setPower(1);
                    H.duckServo.setPosition(1);
                }
            } else {
                H.duckServo.setPosition(0.5);
                //H.collectorMotor.setPower(0);
            }
            
            if (gamepad1.start) {
                
                agl_frwd = heading;
                
            }
            
            //telemetry.addData("wheelPower: ", H.driveMotor[0].getPower());
            //telemetry.addData("wheelPower: ", H.driveMotor[1].getPower());
            //telemetry.addData("wheelPower: ", H.driveMotor[2].getPower());
            //telemetry.addData("wheelPower: ", H.driveMotor[3].getPower());
            //telemetry.addData("barrier mode: ", barrierMode);
            //telemetry.addData("direction: ", traversingForwards);
            telemetry.addData("heading", H.heading);
            telemetry.addData("heading lock", headingLock);
            telemetry.update();
            
        }
        
        pool.shutdownNow();
        
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
    
}
