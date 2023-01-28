package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.concurrent.TimeUnit;

import static java.lang.Thread.sleep;

public class RobotHardware implements Runnable{
    /**
     *  to use RobotHardware use this:
     *
     *      RobotHardware H = new RobotHardware();
     *
     *  then run this in runOpMode()
     *
     *      H.init(hardwareMap)
     *
     *  if you want to read a sensor value or change motor speed use this:
     *
     *      H.[sensor/motor name].[function name]([var 1], [var 2] ...);
     */
    
    public HardwareMap hardwareMap;
    LinearOpMode opMode;

    ////////////////////////////// Constants //////////////////////////////

    public final double COUNTS_PER_REVOLUTION_CORE = 288;
    public final DcMotor.Direction[] MOTOR_DIRECTION = {DcMotor.Direction.FORWARD, DcMotor.Direction.REVERSE, DcMotor.Direction.REVERSE, DcMotor.Direction.FORWARD};
    
    public final double LAUNCH_SERVO_MAX = 0.17;
    public final double LAUNCH_SERVO_MIN = 0.02;
    public final int LAUNCH_SERVO_DELAY = 100;
    public final double LAUNCH_REPEAT_DELAY = 0.25;
    public final double LAUNCH_MAX_SPEED = 0.88;
    
    public final double GRABBER_SERVO_MAX = 0.69;
    public final double GRABBER_SERVO_MIN = 0.22;
    
    private final double COUNTS_PER_REVOLUTION = 560;
    private final double WHEEL_DIAMETER_INCHES = 4.0;
    public final double COUNTS_PER_INCH = COUNTS_PER_REVOLUTION / (WHEEL_DIAMETER_INCHES * Math.PI);
    public final double MINIMUM_MOTOR_POWER = 0.1;
    
    public final double EXP_BASE = 20;
    public final double INITIAL_VALUE = 0.05;
    public final double STICK_DEAD_ZONE = 0.1;
    public final double POWER_FOLLOW_INCREMENT = 0.05;
    public final double ROOT_TWO = Math.sqrt(2);
    
    final long TARGET_LOOP_DURATION = 100;
    
    public final String VUFORIA_KEY = "AXfJetz/////AAABmfTftTQRKUq2u+iCzbuFm2wKhp5/qubTF+6xF9VBwMBiVi2lCwJbNrIAVofnUKke4/MjFtZROHGeelAgbQx6MjYX+qdX4vRB5z2PboepftoqvoZy3irQKQ2aKqNSbpN72hI/tI2wluN0xqC6KThtMURH0EuvUf8VcGDfmuXiA/uP00/2dsYhIMhxBJCmBq0AG5jMWi8MnHJDZwnoYLdcliKB7rvNTUDbf1fzxRzf9QHgB2u+invzPou7q8ncAsD5GdXFfA/CiYmR65JKXDOE0wHoc8FxvrzUIRCQ2geSypo7eY5q/STJvqPmjoj33CQFHl0hKMx05QwwsABdlIZvfLLbjA3VH2HO4dcv+OOoElws";
    
    public static double headingSave = 0;
    private long loopStart;
    
    ////////////////////////////// Toggles //////////////////////////////
    
    boolean XYEncoderEnable = false;
    boolean cameraEnable = false;
    
    ////////////////////////////// Output variables //////////////////////////////
    
    public double heading = 0;
    public double rightDistance = 0;
    public double leftDistance = 0;
    public double frontDistance = 0;
    public int[] driveEncoder = {0, 0, 0, 0};
    public double armVoltage = 0;
    public double rampPower = 0.5;
    public double[] range = {0,0,0,0};
    
    ////////////////////////////// Sensors //////////////////////////////

    public DistanceSensor leftRange;
    public DistanceSensor rightRange;
    public DistanceSensor frontRange;
    public Rev2mDistanceSensor leftTOF;
    public Rev2mDistanceSensor frontTOF;
    //public AnalogInput armAngle;
    public static BNO055IMU      imu;
    public static BNO055IMU      imu1;
    public Orientation    angles;
    public Orientation    angles1;
    public WebcamName webcam;
    public int monitorViewId;
    public DcMotor yEncoder;
    public DcMotor xEncoder;
    public ElapsedTime runtime = new ElapsedTime();
    public DigitalChannel liftStop;
    public DigitalChannel[] rampStop = new DigitalChannel[2];
    public MB1242Ultrasonic[] MB1242 = new MB1242Ultrasonic[4];
    final int[] MB1242_SENSOR_OFFSET = {6,-1,6,1};
    public boolean newRangeDataFlag = false;
    public DigitalChannel[] wheelTriggers = new DigitalChannel[8];

    ////////////////////////////// Motors //////////////////////////////
    
    public DcMotor        collectorMotor;
    public DcMotor        liftMotor;
    public DcMotor[]      driveMotor = new DcMotor[4];
    public Servo[]        wheelLift = new Servo[4];
    public Servo          duckServo;
    public Servo          rampServo;
    public Servo          collectorServo;
    public Servo          clawServo;
    public Servo          distancePlatform;

    public void init(HardwareMap HM, LinearOpMode telOp) {
        
        runtime.reset();
        
        hardwareMap = HM;
        this.opMode = telOp;
        
        ////////////////////////////// Hardware Map //////////////////////////////

        driveMotor[0] = HM.get(DcMotor.class, "FL_Motor");
        driveMotor[1] = HM.get(DcMotor.class, "FR_Motor");
        driveMotor[2] = HM.get(DcMotor.class, "RL_Motor");
        driveMotor[3] = HM.get(DcMotor.class, "RR_Motor");
        liftMotor   = HM.get(DcMotor.class, "Lift_Motor");
    
    
        //wheelLift[0] = HM.get(Servo.class, "FL_Servo");
        //wheelLift[1] = HM.get(Servo.class, "FR_Servo");
        //wheelLift[2] = HM.get(Servo.class, "RL_Servo");
        //wheelLift[3] = HM.get(Servo.class, "RR_Servo");
        //duckServo = HM.get(Servo.class, "D_Servo");
        //rampServo = HM.get(Servo.class, "R_Servo");
        clawServo = HM.get(Servo.class, "C_Servo");
        //collectorServo = HM.get(Servo.class, "C_Servo");
        //distancePlatform = HM.get(Servo.class, "Distance_Servo");
    
        if (cameraEnable) {
            webcam = HM.get(WebcamName.class, "Webcam 1");
            monitorViewId = HM.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", HM.appContext.getPackageName());
        }
        
        MB1242[0] = HM.get(MB1242Ultrasonic.class, "MB0");
        MB1242[1] = HM.get(MB1242Ultrasonic.class, "MB1");
        MB1242[2] = HM.get(MB1242Ultrasonic.class, "MB2");
        MB1242[3] = HM.get(MB1242Ultrasonic.class, "MB3");
        //leftRange = HM.get(DistanceSensor.class, "L_Range");
        //rightRange = HM.get(DistanceSensor.class, "R_Range");
        frontRange = HM.get(DistanceSensor.class, "F_Range");
        imu = HM.get(BNO055IMU.class, "imu");
        //imu1 = HM.get(BNO055IMU.class, "imu1");
    
        if (XYEncoderEnable) {
            yEncoder = HM.get(DcMotor.class, "Y_Motor");
            xEncoder = HM.get(DcMotor.class, "X_Motor");
    
            yEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            yEncoder.setDirection(DcMotor.Direction.FORWARD);
            xEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            xEncoder.setDirection(DcMotor.Direction.REVERSE);
        }
    
        //liftStop = HM.get(DigitalChannel.class, "Lift_Digital");
        
        //rampStop[0] = HM.get(DigitalChannel.class, "RampB_Digital");
        //rampStop[1] = HM.get(DigitalChannel.class, "RampT_Digital");
        
        //for (int i = 0; i < wheelTriggers.length; i++) {
        //    wheelTriggers[i] = HM.get(DigitalChannel.class, i+"_Digital");
        //}

        ////////////////////////////// Parameters //////////////////////////////

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile  = "BNO055IMUCalibration.json";
        imu.initialize(parameters);
        //imu1.initialize(parameters);

        //leftTOF = (Rev2mDistanceSensor) leftRange;
        frontTOF = (Rev2mDistanceSensor) frontRange;
    
        
        for (int i = 3; i >= 0; i--) {
            driveMotor[i].setDirection(MOTOR_DIRECTION[i]);
            driveMotor[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveMotor[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveMotor[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    
    public void run() {
        
        while (!isStopRequested()) {
            
            loopStart = runtime.time(TimeUnit.MILLISECONDS);
    
            //for (MB1242Ultrasonic mb1242Ultrasonic : MB1242) mb1242Ultrasonic.range();
    
            heading = getheading();
            
            //rightDistance = rightTOF.getDistance(DistanceUnit.INCH);
            //leftDistance = leftTOF.getDistance(DistanceUnit.INCH);
            frontDistance = frontTOF.getDistance(DistanceUnit.INCH);
            
            if ((loopStart + TARGET_LOOP_DURATION-5) > runtime.time(TimeUnit.MILLISECONDS)) {
                Log.d("SensLoop","" + (runtime.time(TimeUnit.MILLISECONDS)-loopStart));
                try {
                    Thread.sleep(TARGET_LOOP_DURATION - 5 - (runtime.time(TimeUnit.MILLISECONDS)-loopStart));
                } catch (Exception e) {
                    Log.e("SensLoop", e.getMessage());
                }
                //while ((loopStart + TARGET_LOOP_DURATION-5) > runtime.time(TimeUnit.MILLISECONDS)) {
                //    opMode.idle();
                //}
            } else {
                Log.w("SensLoop", "loop duration exceeds target time frame");
            }
//            for (int i = 0; i < MB1242.length; i++) {
//                range[i] = MB1242[i].inches() + MB1242_SENSOR_OFFSET[i];
//                newRangeDataFlag = true;
//            }
//            try {
//                Thread.sleep(5);
//            } catch (Exception e) {
//                Log.e("SensLoop", e.getMessage());
//            }
            //for (int i = 3; i >= 0; i--) driveEncoder[0] = driveMotor[0].getCurrentPosition();
            //armVoltage = armAngle.getVoltage();
    
        }
        
    }

    public double getheading() {
        // returns a value between -180 and 180, forwards is 0 when initialised
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //angles1 = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //return (angles.firstAngle + angles1.firstAngle)/2;
        return (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) + headingSave);// + AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle))/2;
    }
    
    public void saveHeading() {
        headingSave = getheading();
    }
    
    public boolean isStopRequested() {
        return opMode.isStopRequested();
    }
    
    public void setCameraEnable(boolean cameraEnable) {
        
        this.cameraEnable = cameraEnable;
    }
    
    public void setXYEncoderEnable(boolean XYEncoderEnable) {
        
        this.XYEncoderEnable = XYEncoderEnable;
    }
    
}
