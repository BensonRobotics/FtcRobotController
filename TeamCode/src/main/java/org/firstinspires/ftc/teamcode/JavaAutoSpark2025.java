package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Autonomous
public class
JavaAutoSpark2025 extends LinearOpMode {
    DcMotorEx frontLeftMotor;
    DcMotorEx frontRightMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx backRightMotor;
    CRServo wheelServo;
    CRServo ascendServo3;
    DcMotorEx armMotor;
    DcMotorEx armAngleMotor;
    DcMotorEx ascend;
    DcMotorEx ascend2;
    final float NEW_P_DRIVE = 0.75F;
    final float NEW_I_DRIVE = 0.2F;
    final float NEW_D_DRIVE = 0.1F;
    final float NEW_F_DRIVE = 10.0F;
    final float NEW_P_TURN = 0.5F;
    final short ascendCurrentAlert = 2500;
    final short armCurrentAlert = 2500;
    final short armAngleCurrentAlert = 2500;
    final float driveSpeedLimit = 1F;
    final float turnSpeedLimit = 0.5F;
    final double driveStepsPerMM = 537.7 / (104.0 * Math.PI);
    ElapsedTime runtime = new ElapsedTime();
    int autoStageNum = 0;
    double botHeading;
    // Group drive motors in an array
    final DcMotorEx[] driveMotors = new DcMotorEx[] {
            frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor
    };

    // Group all motors in an array
    final DcMotorEx[] allMotors = new DcMotorEx[] {
            frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, ascend, ascend2, armMotor, armAngleMotor
    };

    // Group all motor hardware names in an array
    final String[] motorHardwareNames = {
            "front_left_drive", "front_right_drive", "back_left_drive", "back_right_drive", "ascend", "ascend2", "armMotor", "armAngleMotor"
    };

    @Override
    public void runOpMode() throws InterruptedException {

        // Assign all motors to hardware map
        for (int i = 0; i < allMotors.length; i++) {
            allMotors[i] = hardwareMap.get(DcMotorEx.class, motorHardwareNames[i]);
        }

        // Assign our devices
        // Make sure your ID's match your configuration
        wheelServo = hardwareMap.get(CRServo.class, "wheelServo");
        ascendServo3 = hardwareMap.get(CRServo.class, "ascendServo3");

        // Set PIDF values for all drive motors
        for (DcMotorEx motor : driveMotors) {
            // Apply drive PIDF coefficients
            motor.setVelocityPIDFCoefficients(NEW_P_DRIVE,NEW_I_DRIVE,NEW_D_DRIVE,NEW_F_DRIVE);
        }

        // Reverse motors that are otherwise backwards
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        ascend.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        // Without this, the REV Hub's orientation is assumed to be logo up USB forward
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        // Set all motors to brake mode, reset encoders, and run using encoders
        for (DcMotorEx motor : allMotors) {
            // For example, setting brake mode and encoder run mode:
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Set motor current thresholds
        ascend.setCurrentAlert(ascendCurrentAlert, CurrentUnit.MILLIAMPS);
        armMotor.setCurrentAlert(armCurrentAlert, CurrentUnit.MILLIAMPS);
        armAngleMotor.setCurrentAlert(armAngleCurrentAlert, CurrentUnit.MILLIAMPS);

        // Reset runtime, fully initialized!
        runtime.reset();
        telemetry.speak("Initialized!");
        telemetry.update();

        // Play button is pressed
        waitForStart();

        // OpMode started!
        telemetry.speak("Started!");
        telemetry.update();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            botHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            switch (autoStageNum) {
                case 0:
                    updateMoveWithPositions(0, 100, 0.5, 0, 0,0,0);
                    if (areMotorsFree()) {
                    autoStageNum = 1;
                    runtime.reset();
                    }
                    break;
                case 1:
                    if (runtime.seconds() > 0.5) {
                        updateMoveWithPositions(0, 0, 0.5, 0, 0, 0, 0);
                        if (areMotorsFree()) {
                            autoStageNum = 2;
                            runtime.reset();
                        }
                    }
                    break;
            }
        }

    }
    private void updateMoveWithPositions (int mmX, int mmY, double drivePower, int targetYaw,
                                          int armAngle, double ascendHeight, double armHeight) {
        // Calculate angle and magnitude from step values
        double driveAngle = Math.atan2(mmX, mmY);
        double driveDistance = Math.hypot(mmX, mmY);
        // Don't ask me why this works
        driveDistance *= 1.5;
        double error = Math.IEEEremainder(targetYaw - botHeading, 360.0);
        error *= NEW_P_TURN;
        int rotation = (int) Math.max(Math.abs(error), turnSpeedLimit * 180 * NEW_P_TURN);
        // The evil code for calculating motor powers
        // Desmos used to troubleshoot directions without robot
        // https://www.desmos.com/calculator/3gzff5bzbn
        double frontLeftBackRightMotors = driveStepsPerMM * driveDistance * Math.sin(driveAngle - botHeading + 0.25 * Math.PI);
        double frontRightBackLeftMotors = driveStepsPerMM * driveDistance * -Math.sin(driveAngle - botHeading - 0.25 * Math.PI);
        // Set all motor target positions
        frontLeftMotor.setTargetPosition((int) (frontLeftBackRightMotors + rotation));
        backLeftMotor.setTargetPosition((int) (frontRightBackLeftMotors + rotation));
        frontRightMotor.setTargetPosition((int) (frontRightBackLeftMotors - rotation));
        backRightMotor.setTargetPosition((int) (frontLeftBackRightMotors - rotation));

        for (DcMotorEx motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(drivePower);
        }
    }
    private boolean areMotorsFree() {
        for (DcMotorEx motor : allMotors) {
            if (motor.isBusy()) {
                return false;
            }
        }
        return true;
    }
}
