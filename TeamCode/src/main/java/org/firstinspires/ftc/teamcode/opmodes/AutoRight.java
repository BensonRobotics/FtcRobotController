package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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

@Autonomous(name="Robot: Auto Right", group="Robot")
public class AutoRight extends LinearOpMode {

    final
    int targetDistance = -2715; // number of clicks to move to left, to get samples
    int shortDistanceA = -250;
    int shortDistanceTwoA = 250;
    int targetDistanceTwo = 2715; // number of clicks to move
    final int shortDistance = 250; //number of clicks to move sideways
    final int shortDistanceTwo = -250; ;
    final double VELOCITY = 500.0; // number of clicks per second
    public static final double NEW_POS_P_DRIVE = 1.0;

    public void runOpMode(){
        DcMotorEx frontLeftMotor;
        DcMotorEx frontRightMotor;
        DcMotorEx backLeftMotor;
        DcMotorEx backRightMotor;
        //DcMotorEx grabberMotor;
        // Declare our motors
        // Make sure your ID's match your configuration
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        //grabberMotor = hardwareMap.get(DcMotorEx.class, "grabberMotor");

        //frontLeftMotor.setPositionPIDFCoefficients(NEW_POS_P_DRIVE);
        //frontRightMotor.setPositionPIDFCoefficients(NEW_POS_P_DRIVE);
        //backLeftMotor.setPositionPIDFCoefficients(NEW_POS_P_DRIVE);
        //backRightMotor.setPositionPIDFCoefficients(NEW_POS_P_DRIVE);

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Calculations to convert linear distance in mm to encoder steps
        double stepsPerMMDrive = 537.7/104.0*Math.PI;
        // Set targetSteps to distance in MM, converted to encoder steps
        // (int) casts float value to int for rounding down
        int targetSteps = (int) (stepsPerMMDrive*1000);

        // Zero all motor encoders
        frontLeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //telemetry.addData(" I am here", 10 );
       // telemetry.update();
        waitForStart();

        //turning left
        frontLeftMotor.setTargetPosition(shortDistanceTwoA);
        backLeftMotor.setTargetPosition(shortDistanceA);
        frontRightMotor.setTargetPosition(shortDistanceA);
        backRightMotor.setTargetPosition(shortDistanceTwoA);

        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //velocity set

        frontLeftMotor.setVelocity(VELOCITY);
        backLeftMotor.setVelocity(VELOCITY);
        frontRightMotor.setVelocity(VELOCITY);
        backRightMotor.setVelocity(VELOCITY);

        while ( opModeIsActive())

        waitForStart();

        //going backwards
        frontLeftMotor.setTargetPosition(shortDistanceTwo);
        backLeftMotor.setTargetPosition(shortDistance);
        frontRightMotor.setTargetPosition(shortDistance);
        backRightMotor.setTargetPosition(shortDistanceTwo);

        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //velocity set
        frontLeftMotor.setVelocity(VELOCITY);
        backLeftMotor.setVelocity(VELOCITY);
        frontRightMotor.setVelocity(VELOCITY);
        backRightMotor.setVelocity(VELOCITY);

        while ( opModeIsActive()          &&
                frontLeftMotor.isBusy()   &&
                backLeftMotor.isBusy()    &&
                frontRightMotor.isBusy()  &&
                backRightMotor.isBusy())
        {
            telemetry.addData(" frontleft position", frontLeftMotor.getCurrentPosition());
            telemetry.addData(" frontright position", frontRightMotor.getCurrentPosition());
            telemetry.addData("frontLeftMotor", "I'm here yay");
            telemetry.addData("backleft position", backLeftMotor.getCurrentPosition());
            telemetry.addData ( "backright position", backRightMotor.getCurrentPosition());
            telemetry.update();

        }

        //set them to zero again
        frontLeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Set all motor target positions
        frontLeftMotor.setTargetPosition(targetDistanceTwo);
        backLeftMotor.setTargetPosition(targetDistanceTwo);
        frontRightMotor.setTargetPosition(targetDistanceTwo);
        backRightMotor.setTargetPosition(targetDistanceTwo);

        // Tell all motors to run to target position
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Set all motor power levels
        frontLeftMotor.setVelocity(VELOCITY);
        backLeftMotor.setVelocity(VELOCITY);
        frontRightMotor.setVelocity(VELOCITY);
        backRightMotor.setVelocity(VELOCITY);


        //while (opModeIsActive()&&(frontLeftMotor.isBusy() || backLeftMotor.isBusy () || frontRightMotor.isBusy () || backRightMotor.isBusy ()))
        while ( opModeIsActive()          &&
                frontLeftMotor.isBusy()   &&
                backLeftMotor.isBusy()    &&
                frontRightMotor.isBusy()  &&
                backRightMotor.isBusy())
        {
            telemetry.addData(" frontleft position", frontLeftMotor.getCurrentPosition());
            telemetry.addData(" frontright position", frontRightMotor.getCurrentPosition());
            telemetry.addData("frontLeftMotor", "I'm here yay");
            telemetry.addData( "backleft position", backLeftMotor.getCurrentPosition());
            telemetry.addData( "backright position", backRightMotor.getCurrentPosition());
            telemetry.update();

           // if (frontLeftMotor.getCurrentPosition() >
                   // targetSteps)
           // if (frontLeftMotor.getCurrentPosition()
           // >=targetDistance)
           // {
           //     telemetry.addData("frontLeftMotor", "I'm here");
           //     telemetry.update();
           //     break;
           //

        }

        // Shuts everything down
        frontLeftMotor.setVelocity(0);
        backLeftMotor.setVelocity(0);
        frontRightMotor.setVelocity(0);
        backRightMotor.setVelocity(0);

    }
}

