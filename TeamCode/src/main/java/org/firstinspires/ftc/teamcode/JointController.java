package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class JointController {
    private final DcMotorEx[] jointMotors;
    private final Servo[] jointServos;
    private final double[] angleToOutput;
    private final double[] servoOffsets;

    public JointController(DcMotorEx[] jointMotors, Servo[] jointServos,
                           double[] angleToOutput, double[] servoOffsets) {
        this.jointMotors = jointMotors;
        this.jointServos = jointServos;
        this.angleToOutput = angleToOutput;
        this.servoOffsets = servoOffsets;
    }

    public void initialize(HardwareMap hardwareMap, String[] jointNames, double[] motorPPROrServoRange) {
        for (int i = 0; i < jointNames.length; i++) {
            jointMotors[i] = hardwareMap.tryGet(DcMotorEx.class, jointNames[i]);
            jointServos[i] = hardwareMap.tryGet(Servo.class, jointNames[i]);
            if (jointMotors[i] != null) {
                jointMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                jointMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                jointMotors[i].setTargetPosition(0);
                jointMotors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
                jointMotors[i].setPower(1);
                angleToOutput[i] = motorPPROrServoRange[i] / (2 * Math.PI);
                servoOffsets[i] = 0;
            } else if (jointServos[i] != null) {
                jointServos[i].setPosition(0.5);
                angleToOutput[i] = 1 / motorPPROrServoRange[i];
                servoOffsets[i] = motorPPROrServoRange[i] / 2;
            }
        }
    }

    public double approxGoBildaMotorPPR(int RPM) {
        // Regressed using desmos, https://www.desmos.com/calculator/0trokx5reb
        // Probably not used for much because you need precision for this stuff
        return 162182.683 * Math.pow(RPM, -0.995121);
    }

    public void applyJointAngles(double[] jointAngles) {
        // Apply joint angle to motor or servo
        for (int i = 0; i < jointAngles.length; i++) {
            if (jointMotors[i] != null) {
                jointMotors[i].setTargetPosition((int) (jointAngles[i] * angleToOutput[i]));
            } else if (jointServos[i] != null) {
                jointServos[i].setPosition((float) ((jointAngles[i] + servoOffsets[i]) *
                        angleToOutput[i]));
            }
        }
    }

    public double[] readJointAngles() {
        // Read motor angles from motors and output in radians
        // Could get used for pathing algorithm or something
        double[] angles = new double[jointMotors.length];
        for (int i = 0; i < jointMotors.length; i++) {
            if (jointMotors[i] != null) {
                angles[i] = jointMotors[i].getCurrentPosition() / angleToOutput[i];
            } else if (jointServos[i] != null) {
                angles[i] = (jointServos[i].getPosition() / angleToOutput[i]) - servoOffsets[i];
            }
        }
        return angles;
    }
}
