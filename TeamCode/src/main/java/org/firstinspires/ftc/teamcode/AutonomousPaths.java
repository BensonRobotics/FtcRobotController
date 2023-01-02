package org.firstinspires.ftc.teamcode;

import android.util.Log;

import java.util.concurrent.TimeUnit;

import static android.content.ContentValues.TAG;

public class AutonomousPaths {
    
    MecanumWheelDriverV2 drive;
    RobotHardware H;
    autonomous auto;
    
    final String[][] menus = {{"Field Side:", "Red", "Blue"},{"Autonomous Path:","Duck -A","Drop -B","Park -B","Park -A","Duck Only -A", "Square"}, {"Path Selected"}};
    
    AutonomousPaths(MecanumWheelDriverV2 drive, RobotHardware H, autonomous auto) {
        
        this.drive = drive;
        this.H = H;
        this.auto = auto;
        
    }
    
    void runPath(int side, int path) {
    
        Log.d(TAG,"path: " + path + " side: " + side);
        if (side == 0) { // red
            
            switch (path) {
                case 0:
                    RedDuckA();
                    break;
                case 1:
                    RedDropB();
                    break;
                case 2:
                    RedParkB();
                    break;
                case 3:
                    RedParkA();
                    break;
                case 4:
                    RedDuckOnly();
                    break;
                case 5:
                    Square();
                    break;
                default:
                    break;
            }
            
        } else { // blue
    
            switch (path) {
                case 0:
                    BlueDuckA();
                    break;
                case 1:
                    BlueDropB();
                    break;
                case 2:
                    BlueParkB();
                    break;
                case 3:
                    BlueParkA();
                    break;
                case 4:
                    BlueDuckOnly();
                    break;
                case 5:
                    Square();
                    break;
                default:
                    break;
            }
            
        }
        
    }
    
    void Square() {
    
        drive.StrafeDistanceMove(0,40,0.5,1);
        drive.HeadingRotate(-90, 0.5, 1);
        drive.startActions();
        drive.waitForMoveDone();
    
        drive.StrafeDistanceMove(-90,40,0.5,1);
        drive.HeadingRotate(180, 0.5, 1);
        drive.startActions();
        drive.waitForMoveDone();
    
        drive.StrafeDistanceMove(180,40,0.5,1);
        drive.HeadingRotate(90, 0.5, 1);
        drive.startActions();
        drive.waitForMoveDone();
        
        drive.StrafeDistanceMove(90,40,0.5,1);
        drive.HeadingRotate(0, 0.5, 1);
        drive.startActions();
        drive.waitForMoveDone();
        
    }
    
    void BlueDuckA() {
        
        drive.StrafeDistanceMove(53,30,0.5,1);
        drive.startActions();
        
        auto.deployRamp();
        
        drive.waitForMoveDone();
        auto.dropFreight();
        drive.StrafeDistanceMove(-90,45,0.5,1);
        drive.HeadingRotate(-90, 0.5, 1);
        drive.startActions();
        drive.waitForMoveDone();
        drive.HeadingRotate(-90,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
    
        long startTime = H.runtime.time(TimeUnit.MILLISECONDS);
        while (H.rightDistance > 8 && !auto.isStopRequested() && startTime + 1000 < H.runtime.time(TimeUnit.MILLISECONDS)) {
            auto.telemetry.addLine("dis: " + H.rightDistance);
            auto.telemetry.update();
            drive.StrafePowerMove(-90, 0.4,1);
            drive.startActions();
        }
        drive.stop();
        drive.StrafePowerMove(-90, 0.15,1);
        
        auto.spinDuck();
        drive.stop();
        
        drive.StrafeDistanceMove(-10, 23, 0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
        
    }
    
    void RedDuckA() {
        
        drive.StrafeDistanceMove(-53,30,0.5,1);
        drive.startActions();
    
        auto.deployRamp();
    
        drive.waitForMoveDone();
        auto.dropFreight();
        drive.StrafeDistanceMove(90,45,0.5,1);
        drive.HeadingRotate(180, 0.5, 1);
        drive.startActions();
        drive.waitForMoveDone();
        drive.HeadingRotate(180,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
        
        long startTime = H.runtime.time(TimeUnit.MILLISECONDS);
        while (H.frontDistance > 8 && !auto.isStopRequested() && startTime + 1000 < H.runtime.time(TimeUnit.MILLISECONDS)) {
            auto.telemetry.addLine("dis: " + H.frontDistance);
            auto.telemetry.update();
            drive.StrafePowerMove(0, 0.4,1);
            drive.startActions();
        }
        drive.stop();
        //drive.StrafePowerMove(0, 0.15,1);
    
        auto.spinDuck();
        //drive.stop();
    
        drive.StrafeDistanceMove(10, 23, 0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
        
    }
    
    void BlueDropB() {
        
        drive.StrafeDistanceMove(5,39,0.5,1);
        drive.HeadingRotate(-90,0.5,1);
        drive.startActions();
        
        auto.deployRamp();
        
        drive.waitForMoveDone();
        
        drive.HeadingRotate(-90,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
        
        auto.dropFreight();
    
    
        drive.HeadingRotate(-90,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
        drive.StrafeDistanceMove(180, 19, 0.5,1);
        H.collectorServo.setPosition(1);
        drive.startActions();
        drive.waitForMoveDone();
        drive.HeadingRotate(-90,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
        drive.StrafeDistanceMove(90,41,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
        
    }
    
    void RedDropB() {
        
        drive.StrafeDistanceMove(-5,39,0.5,1);
        drive.HeadingRotate(90,0.5,1);
        drive.startActions();
    
        auto.deployRamp();
    
        drive.waitForMoveDone();
    
        drive.HeadingRotate(90,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
    
        auto.dropFreight();
    
    
        drive.HeadingRotate(90,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
        drive.StrafeDistanceMove(180, 19, 0.5,1);
        H.collectorServo.setPosition(1);
        drive.startActions();
        drive.waitForMoveDone();
        drive.HeadingRotate(90,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
        drive.StrafeDistanceMove(-90,41,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
        
    }
    
    void BlueParkB() {
    
        drive.StrafeDistanceMove(5,14,0.5,1);
        drive.startActions();
        H.collectorServo.setPosition(1);
        drive.waitForMoveDone();
        drive.HeadingRotate(-90,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
    
        //drive.HeadingRotate(-90,0.5,1);
        drive.StrafeDistanceMove(90,38,0.4,1);
        drive.startActions();
        drive.waitForMoveDone();
    }
    
    void RedParkB() {
    
        drive.StrafeDistanceMove(-5,14,0.5,1);
        drive.startActions();
        H.collectorServo.setPosition(1);
        drive.waitForMoveDone();
        drive.HeadingRotate(90,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
    
        //drive.HeadingRotate(-90,0.5,1);
        drive.StrafeDistanceMove(-90,38,0.4,1);
        drive.startActions();
        drive.waitForMoveDone();
        
    }
    
    void BlueParkA() {
    
        drive.StrafeDistanceMove(0,40,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
    
    }
    
    void RedParkA() {
        
        drive.StrafeDistanceMove(0,40,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
        
    }
    
    void BlueDuckOnly() {
    
        drive.StrafeDistanceMove(-55,31,0.5,1);
        drive.HeadingRotate(-90, 0.5, 1);
        drive.startActions();
        drive.waitForMoveDone();
    
        while (H.rightDistance > 12 && !auto.isStopRequested()) {
            drive.StrafePowerMove(180, 0.4,1);
            drive.startActions();
        }
        drive.stop();
    
        auto.spinDuck();
        
        drive.StrafeDistanceMove(0, 24,0.6,1);
        drive.startActions();
        drive.waitForMoveDone();
        
    }
    
    void RedDuckOnly() {
    
        drive.StrafeDistanceMove(55,31,0.5,1);
        drive.HeadingRotate(90, 0.5, 1);
        drive.startActions();
        drive.waitForMoveDone();
    
        while (H.rightDistance > 12 && !auto.isStopRequested()) {
            drive.StrafePowerMove(-180, 0.4,1);
            drive.startActions();
        }
        drive.stop();
    
        auto.spinDuck();
    
        drive.StrafeDistanceMove(0, 24,0.6,1);
        drive.startActions();
        drive.waitForMoveDone();
        
    }
    
}
