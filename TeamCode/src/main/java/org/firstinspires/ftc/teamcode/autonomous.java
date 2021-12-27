/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@Autonomous(name="Autonomous", group="Linear Opmode")
public class autonomous extends LinearOpMode {

    //private static final String VUFORIA_KEY = "AXfJetz/////AAABmfTftTQRKUq2u+iCzbuFm2wKhp5/qubTF+6xF9VBwMBiVi2lCwJbNrIAVofnUKke4/MjFtZROHGeelAgbQx6MjYX+qdX4vRB5z2PboepftoqvoZy3irQKQ2aKqNSbpN72hI/tI2wluN0xqC6KThtMURH0EuvUf8VcGDfmuXiA/uP00/2dsYhIMhxBJCmBq0AG5jMWi8MnHJDZwnoYLdcliKB7rvNTUDbf1fzxRzf9QHgB2u+invzPou7q8ncAsD5GdXFfA/CiYmR65JKXDOE0wHoc8FxvrzUIRCQ2geSypo7eY5q/STJvqPmjoj33CQFHl0hKMx05QwwsABdlIZvfLLbjA3VH2HO4dcv+OOoElws";
    //private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    //private static final String LABEL_FIRST_ELEMENT = "Skystone";

    //private VuforiaLocalizer vuforia;
    //private TFObjectDetector tfod;

    //private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private ElapsedTime         runtime  = new ElapsedTime();
    private RobotHardware       H        = new RobotHardware();
    private MecanumWheelDriverV2  drive    = new MecanumWheelDriverV2(H);
    private ExecutorService     pool     = Executors.newFixedThreadPool(1);
    
    

    @Override
    public void runOpMode() {
    
        H.init(hardwareMap, this);
    
        //aimBot = new AimBot(H, drive, pool, this);
        //RingCounter         ring     = new RingCounter(aimBot.vuforia);
    
        //H.grabberServo.setPosition(H.GRABBER_SERVO_MIN);
    
        waitForStart();
        
        drive.StrafeDistanceMove(0,10,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
        drive.StrafeDistanceMove(90,5,0.5,1);
        drive.startActions();
        drive.waitForMoveDone();
    
    }
   
    
    private void waitForMoveDone() {
        
        sleep(50);
        
        while (!isStopRequested()) {
            idle();
        }
        
        }

}