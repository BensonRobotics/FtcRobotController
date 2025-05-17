package org.firstinspires.ftc.teamcode.robotarm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Controller {
    private final DcMotorEx[] jointMotors;
    private final Servo[] jointServos;
    private final double[] angleToOutput;
    private final float[] angleOffsets;
    public Controller(HardwareMap hardwareMap) {
        int numberOfJoints = Config.NUM_OF_JOINTS;
            this.jointMotors = new DcMotorEx[numberOfJoints];
            this.jointServos = new Servo[numberOfJoints];
            this.angleToOutput = new double[numberOfJoints];
            this.angleOffsets = new float[numberOfJoints];
            for (int i = 0; i < numberOfJoints; i++) {
                this.jointMotors[i] = hardwareMap.tryGet(DcMotorEx.class, Config.JOINT_NAMES[i]);
                this.jointServos[i] = hardwareMap.tryGet(Servo.class, Config.JOINT_NAMES[i]);
                if (this.jointMotors[i] != null) {
                    this.jointMotors[i].setPositionPIDFCoefficients(Config.JOINT_MOTOR_PROPORTIONALS[i]);
                    this.jointMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    this.jointMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    this.jointMotors[i].setTargetPosition(0);
                    this.jointMotors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    this.jointMotors[i].setPower(1);
                    this.angleToOutput[i] = Config.JOINT_MOTOR_PPRS[i] / (2 * Math.PI);
                    this.angleOffsets[i] = 0;
                    if (Config.JOINTS_REVERSED[i]) {
                        this.jointMotors[i].setDirection(DcMotorSimple.Direction.REVERSE);
                    }
                } else if (this.jointServos[i] != null) {
                    this.jointServos[i].setPosition(0.5);
                    this.angleToOutput[i] = 1.0 / Math.toRadians(Config.JOINT_SERVO_RANGES[i]);
                    this.angleOffsets[i] = (float) (Math.toRadians(Config.JOINT_SERVO_RANGES[i]) / 2.0);
                    if (Config.JOINTS_REVERSED[i]) {
                        this.jointServos[i].setDirection(Servo.Direction.REVERSE);
                    }
                }
            }
    }

    public static double approxGoBildaMotorPPR(int RPM) {
        // Regressed using desmos, https://www.desmos.com/calculator/0trokx5reb
        // Probably not used for much because you need precision for this stuff
        // Yeah no don't use this shit
        return 162182.683 * Math.pow(RPM, -0.995121);
    }

    public void applyJointAngles(double[] jointAngles) {
        if (jointAngles.length != jointMotors.length) {
            throw new IllegalArgumentException("Invalid number of joints");
        }
        // Apply joint angle to motor or servo
        for (int i = 0; i < jointAngles.length; i++) {
            if (jointMotors[i] != null) {
                jointMotors[i].setTargetPosition((int) ((jointAngles[i] + angleOffsets[i]) * angleToOutput[i]));
            } else if (jointServos[i] != null) {
                jointServos[i].setPosition((jointAngles[i] + angleOffsets[i]) * angleToOutput[i]);
            }
        }
    }

    public double[] readJointAngles() {
        // Read motor angles from motors and output in radians
        // Could get used for pathing algorithm or something
        double[] angles = new double[jointMotors.length];
        for (int i = 0; i < jointMotors.length; i++) {
            if (jointMotors[i] != null) {
                angles[i] = (jointMotors[i].getCurrentPosition() / angleToOutput[i]) - angleOffsets[i];
            } else if (jointServos[i] != null) {
                angles[i] = (jointServos[i].getPosition() / angleToOutput[i]) - angleOffsets[i];
            }
        }
        return angles;
    }
}
