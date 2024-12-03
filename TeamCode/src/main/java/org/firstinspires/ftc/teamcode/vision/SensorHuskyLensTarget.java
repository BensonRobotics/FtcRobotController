/*
Copyright (c) 2023 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Edward C. Epp
December 2024

Started as FIRST's SensorHuskyLens.java
Modified to identify color objects.
The program will turn and move toward them.
Issue - It does not stop once it nolonger sees the target oject.

*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates how to use the DFRobot HuskyLens.
 *
 * The HuskyLens is a Vision Sensor with a built-in object detection model.  It can
 * detect a number of predefined objects and AprilTags in the 36h11 family, can
 * recognize colors, and can be trained to detect custom objects. See this website for
 * documentation: https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336
 *
 * For detailed instructions on how a HuskyLens is used in FTC, please see this tutorial:
 * https://ftc-docs.firstinspires.org/en/latest/devices/huskylens/huskylens.html
 *
 * This sample illustrates how to detect AprilTags, but can be used to detect other types
 * of objects by changing the algorithm. It assumes that the HuskyLens is configured with
 * a name of "huskylens".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Autonomous(name = "HuskyLens: Target a game piece ", group = "Sensor")

public class SensorHuskyLensTarget extends LinearOpMode {

    private final int READ_PERIOD    =   1;
    private final int DISPLAY_X      = 320;
    private final int OFFSET_X       =  25;
    private final double MOTOR_POWER =   0.2;


    private HuskyLens huskyLens;

    @Override
    public void runOpMode()
    {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        /*
         * This sample rate limits the reads solely to allow a user time to observe
         * what is happening on the Driver Station telemetry.  Typical applications
         * would not likely rate limit.
         */
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

        /*
         * Immediately expire so that the first time through we'll do the read.
         */
        rateLimit.expire();

        /*
         * Basic check to see if the device is alive and communicating.  This is not
         * technically necessary here as the HuskyLens class does this in its
         * doInitialization() method which is called when the device is pulled out of
         * the hardware map.  However, sometimes it's unclear why a device reports as
         * failing on initialization.  In the case of this device, it's because the
         * call to knock() failed.
         */
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        /*
         * The device uses the concept of an algorithm to determine what types of
         * objects it will look for and/or what mode it is in.  The algorithm may be
         * selected using the scroll wheel on the device, or via software as shown in
         * the call to selectAlgorithm().
         *
         * The SDK itself does not assume that the user wants a particular algorithm on
         * startup, and hence does not set an algorithm.
         *
         * Users, should, in general, explicitly choose the algorithm they want to use
         * within the OpMode by calling selectAlgorithm() and passing it one of the values
         * found in the enumeration HuskyLens.Algorithm.
         *
         * Other algorithm choices for FTC might be: OBJECT_RECOGNITION, COLOR_RECOGNITION or OBJECT_CLASSIFICATION.
         */
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        telemetry.update();
        waitForStart();

        /*
         Recognize the color of some game pieces.
         */

        // fire up the motors
        DcMotor leftMotor  = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        DcMotor rightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");

        while(opModeIsActive()) {
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            /*
             * Returns an empty array if no objects are seen.
             */
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);

            // look for the target block
            boolean iSeeBlock = Boolean.FALSE;
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
                telemetry.addData("ID: ", blocks[i].id);

                iSeeBlock = Boolean.FALSE;
                // Take action based on finding a target block
                if (blocks[i].id == 1)
                {
                    iSeeBlock = Boolean.TRUE;
                    // Turn left toward the target block
                    if (blocks[i].x < DISPLAY_X / 2.2 - OFFSET_X)
                    {
                        telemetry.addData("Turn  Left","");
                        leftMotor.setPower(MOTOR_POWER);
                        rightMotor.setPower(MOTOR_POWER);
                    }

                    // Turn right toward the target block
                    else if (blocks[i].x > DISPLAY_X / 2.0 + OFFSET_X)
                    {
                        telemetry.addData("turn Right","");
                        leftMotor.setPower(-MOTOR_POWER);
                        rightMotor.setPower(-MOTOR_POWER);
                    }

                    // Move forward toward the target block
                    else
                    {
                        telemetry.addData("Forward","");
                        leftMotor.setPower(-MOTOR_POWER);
                        rightMotor.setPower(MOTOR_POWER);
                    };
                };
            }

            telemetry.update();

            // Target block not seen so stop
            /*
            if (!iSeeBlock)
            {
                telemetry.addData("Can't see block","");
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }
            */

        }

        // Done so stop
        telemetry.addData("Stop","");
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}
