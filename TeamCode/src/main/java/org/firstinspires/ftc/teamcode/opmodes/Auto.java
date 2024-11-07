package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
Hi, Desi here. I was going to implement some fixes, but I was informed that Ed would be using
this to teach some robotics members programming and that they would be working on the Auto.
I do have some suggestions, though. For starters, since this autonomous will be running to
a different position over and over again, it would be best to put the reset-setTarget-drive code
inside a void, and call that void with new position parameters every time you want to move
the robot. You could program the void to take in individual motor positions and power levels,
or you could challenge yourself and have it intake an angle and power level, and have
trigonometry calculate the motor positions and powers from there. It shouldn't be too hard to
repurpose some Mecanum drive code into setting motor positions instead of powers. Another thing,
when having the code only run through once, keep in mind that once it reaches the bottom, it
doesn't loop back or anything, it actually stops the OpMode and kills all your motors. If you
want it to ever wait until it fully reaches its previous destination, you can have it call the
function, then trap the code in a while loop that is only running while any of the motors are
busy, a boolean value that looks like motorName.isBusy() in your while condition, which,
all together, would look like this: while (frontRightMotor.isBusy() || frontLeftMotor.isBusy() ||
backRightMotor.isBusy() || backLeftMotor.isBusy()) {}
This would keep the code held up there until all of the motors have reached their target, then
it would proceed. You may also want to add a sleep(500);, which would have it hold for another
half second, just to make sure all residual momentum is gone.
 */

@Autonomous
public class Auto extends LinearOpMode {
    double NEW_POS_P_DRIVE = 1.0;
    double stepsPerMMDrive = 537.7 / (104.0 * Math.PI);
    DcMotorEx frontLeftMotor;
    DcMotorEx frontRightMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx backRightMotor;
    IMU imu = hardwareMap.get(IMU.class, "imu");
    double botHeading = 0;
    boolean useFieldCentric = true;

    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        frontLeftMotor.setPositionPIDFCoefficients(NEW_POS_P_DRIVE);
        frontRightMotor.setPositionPIDFCoefficients(NEW_POS_P_DRIVE);
        backLeftMotor.setPositionPIDFCoefficients(NEW_POS_P_DRIVE);
        backRightMotor.setPositionPIDFCoefficients(NEW_POS_P_DRIVE);

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set all motors to brake automatically
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Zero all motor encoders
        frontLeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up USB forward
        imu.initialize(parameters);

        waitForStart();

        driveWithTrigonometry(0,100,0,0.25);
        wait(500);
        driveWithTrigonometry(100,0,0,0.25);
        wait(500);
        driveWithTrigonometry(-100,-100,0,0.25);
        wait(500);
    }
    private void driveWithTrigonometry(int relativeX, int relativeY, double rotation, double power) {
        // Calculate angle and hypotenuse
        double driveAngle = Math.atan2(relativeX, relativeY);
        double driveDistance = Math.hypot(relativeX, relativeY);

        // Run all the motors to target positions
        // Do the loop once, then keep looping until all motors are done
        // I love do-while loops
        do {
            if (useFieldCentric) { // Get the robot rotation value, if using field centric
                botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            }
            // Calculate motor positions
            double frontLeftBackRightMotors = driveDistance * Math.sin(driveAngle + botHeading + 0.25 * Math.PI);
            double frontRightBackLeftMotors = driveDistance * -Math.sin(driveAngle + botHeading - 0.25 * Math.PI);

            // Set all motor target positions
            frontLeftMotor.setTargetPosition((int) (stepsPerMMDrive * frontLeftBackRightMotors + rotation));
            backLeftMotor.setTargetPosition((int) (stepsPerMMDrive * frontRightBackLeftMotors + rotation));
            frontRightMotor.setTargetPosition((int) (stepsPerMMDrive * frontRightBackLeftMotors - rotation));
            backRightMotor.setTargetPosition((int) (stepsPerMMDrive * frontLeftBackRightMotors - rotation));

            // Tell all motors to run to target position
            frontRightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            // Set all motor power levels
            frontRightMotor.setPower(power);
            frontLeftMotor.setPower(power);
            backRightMotor.setPower(power);
            backLeftMotor.setPower(power);
        } while (frontRightMotor.isBusy() || frontLeftMotor.isBusy() || backRightMotor.isBusy() || backLeftMotor.isBusy());
    }
}
