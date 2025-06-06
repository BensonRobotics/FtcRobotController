package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;

// This is a collection of all the functions necessary for lift operation.
public class LiftFunctionReference extends LinearOpMode {

    private DcMotorEx liftMotor = null;

    public void runOpMode() {
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");

        liftMotor.setDirection(DcMotorEx.Direction.REVERSE);

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // This is the maximum current that the lift motor can draw before the IsOverloaded(liftMotorCurrentThreshold) function call will start returning true
        double liftMotorCurrentThreshold = 3000.0;

        waitForStart();

        int liftBottomPosition = GetLiftBottomPosition(liftMotorCurrentThreshold);
        liftMotor.setTargetPosition(liftBottomPosition);
    }

    // Update the lift motor target position based on controller inputs. To use a different control source,
    // just change the conditions for the if statements containing the code to change the desiredPositionIndex var.
    // Included at the bottom of the file is a quick adaptation of this function I made to take the desired target position as an argument.
    private void UpdateLiftMotor(int bottomPosition, double currentThreshold) {
        ArrayList<Integer> liftPositions = new ArrayList<Integer>(3);
        liftPositions.add(bottomPosition);
        liftPositions.add(bottomPosition + 2000);
        liftPositions.add(bottomPosition + 4200);

        if (gamepad2.a || gamepad2.x || gamepad2.y) {
            // Since the variable is initialized to 0, and if we are running this code we know either a, x, or y has been pressed,
            // we can skip the conditional for one of the buttons, in this case a.
            int desiredPositionIndex = 0;
            if (gamepad2.x) {
                desiredPositionIndex = 1;
            } else if (gamepad2.y) {
                desiredPositionIndex = 2;
            }

            int desiredPosition = liftPositions.get(desiredPositionIndex);

            liftMotor.setTargetPosition(desiredPosition);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(1.0);

            if (IsOverloaded(liftMotor, currentThreshold)) {
                liftMotor.setPower(0.0);
            }
        }

        if (IsOverloaded(liftMotor, currentThreshold)) {
            liftMotor.setPower(0.0);
        }

        telemetry.addData("liftMotorPosition", liftMotor.getCurrentPosition());
        telemetry.addData("liftMotorCurrent", liftMotor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Lift motor mode", liftMotor.getMode());

        // this bit of code is for a lift mode using the right stick y to set the lift's power. It can be uncommented to enable this mode
//        double rightStickY = -gamepad2.right_stick_y;
//        if (rightStickY != 0) {
//            liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//            liftMotor.setPower(rightStickY);
//        } else {
//            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
    }

    // Calibrate the lift by slowly running it into the bottom of it's travel, and detecting when the resistance on the motor reaches a certain
    // threshold, indicated by a spike in current draw. Then set the initial position of the lift to the current encoder value, since we know the lift
    // is at the bottom of its travel. This is necessary because of the inevitable drift in the encoders.
    private int GetLiftBottomPosition(double currentThreshold) {
        while (!IsOverloaded(liftMotor, currentThreshold)) {
            liftMotor.setPower(-0.5);
            telemetry.addData("zeroing lift", IsOverloaded(liftMotor, currentThreshold));
            updateTelemetry(telemetry);
        }
        liftMotor.setPower(0.0);

        telemetry.addData("lift bottom position: ", liftMotor.getCurrentPosition());
        return liftMotor.getCurrentPosition();
    }

    private boolean IsOverloaded(DcMotorEx motor, double currentThreshold) {
        return (motor.getCurrent(CurrentUnit.MILLIAMPS) > currentThreshold);
    }


    // To use this function, just replace the function of the same name above with this one, and remove the call to it in the main loop.
    // Then simply call it with the target position index you want and the lift should move to the position corresponding to that index in the list below,
    // as long as I've programmed this right  ( :
//    private void UpdateLiftMotor(int targetPositionIndex, int bottomPosition, double currentThreshold) {
//        ArrayList<Integer> liftPositions = new ArrayList<Integer>(3);
//        liftPositions.add(bottomPosition);
//        liftPositions.add(bottomPosition + 2000);
//        liftPositions.add(bottomPosition + 4200);
//
//
//        int desiredPosition = liftPositions.get(targetPositionIndex);
//
//        liftMotor.setTargetPosition(desiredPosition);
//        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        liftMotor.setPower(1.0);
//
//
//        if (IsOverloaded(liftMotor, currentThreshold)) {
//            liftMotor.setPower(0.0);
//        }
//
//        telemetry.addData("liftMotorPosition", liftMotor.getCurrentPosition());
//        telemetry.addData("liftMotorCurrent", liftMotor.getCurrent(CurrentUnit.MILLIAMPS));
//        telemetry.addData("Lift motor mode", liftMotor.getMode());
//
//        // this bit of code is for a lift mode using the right stick y to set the lift's power. It can be uncommented to enable this mode
////        double rightStickY = -gamepad2.right_stick_y;
////        if (rightStickY != 0) {
////            liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
////            liftMotor.setPower(rightStickY);
////        } else {
////            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        }
//    }
}