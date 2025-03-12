package pedroPathing.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

class robotDimesions {
    static double width = 17.75;
    static double length = 17.625;
    static double armLength = 16.5;
}

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
public class PedroAuto extends OpMode {

    private DcMotorEx armMotor = null;
    private DcMotorEx armAngleMotor = null;
    private DcMotorEx ascend = null;
    private DcMotorEx ascend2 = null;
    private CRServo wheelServo = null;
    private CRServo ascendServo3 = null;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Timer wallSpecWaitTimer;
    private boolean wallSpecWaiting = false;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(10, 63, Math.toRadians(0));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private PathChain initForward, initScore, travelToSpike1, pushSpike1, travelToSpike2, pushSpike2,
            travelToSpike3, pushSpike3, clearWall, wallGrab1, chamberSpec1, wallGrab2, chamberSpec2,
            wallGrab3, chamberSpec3, wallGrab4, chamberSpec4, park;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/

    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our path1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        initForward = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(10.000, 63.000, Point.CARTESIAN),
                                new Point(30.000, 63.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                // Finish raising armMotor after ducking the bar
                .addParametricCallback(0.95, () -> armMotor.setTargetPosition(3600))
                .build();

        /* This is our path4 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        initScore = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(30.000, 63.000, Point.CARTESIAN),
                                new Point(20.000, 63.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        /* This is our path2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        travelToSpike1 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(20.000, 63.000, Point.CARTESIAN),
                                new Point(28.654, 16.755, Point.CARTESIAN),
                                new Point(58.810, 51.066, Point.CARTESIAN),
                                new Point(63.000, 26.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        /* This is our path5 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        pushSpike1 = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(63.000, 26.000, Point.CARTESIAN),
                                new Point(22.000, 26.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        /* This is our path3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        travelToSpike2 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(22.000, 26.000, Point.CARTESIAN),
                                new Point(61.601, 29.372, Point.CARTESIAN),
                                new Point(63.000, 17.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        /* This is our path6 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        pushSpike2 = follower.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(63.000, 17.000, Point.CARTESIAN),
                                new Point(22.000, 17.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        /* This is our path6 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        travelToSpike3 = follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierCurve(
                                new Point(22.000, 17.000, Point.CARTESIAN),
                                new Point(62.651, 23.555, Point.CARTESIAN),
                                new Point(63.000, 11.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();
        /* This is our path6 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        pushSpike3 = follower.pathBuilder()
                .addPath(
                        // Line 8
                        new BezierLine(
                                new Point(63.000, 11.000, Point.CARTESIAN),
                                new Point(22.000, 11.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        /* This is our path6 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        clearWall = follower.pathBuilder()
                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(22.000, 11.000, Point.CARTESIAN),
                                new Point(28.000, 13.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                // Ready to pick up wall specimen
                .addParametricCallback(0.2, () -> armAngleMotor.setTargetPosition(3700))
                .build();
        wallGrab1 = follower.pathBuilder()
                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(28.000, 13.500, Point.CARTESIAN),
                                new Point(25.000, 13.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        /* This is our path6 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        chamberSpec1 = follower.pathBuilder()
                .addPath(
                        // Line 10
                        new BezierCurve(
                                new Point(25.000, 13.500, Point.CARTESIAN),
                                new Point(16.000, 72.000, Point.CARTESIAN),
                                new Point(35.000, 65.500, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();

        /* This is our path6 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        wallGrab2 = follower.pathBuilder()
                .addPath(
                        // Line 11
                        new BezierCurve(
                                new Point(35.000, 65.500, Point.CARTESIAN),
                                new Point(15.541, 73.093, Point.CARTESIAN),
                                new Point(10.500, 16.500, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                // Ready to pick up specimen
                .addParametricCallback(0.1, () -> armAngleMotor.setTargetPosition(3800))
                .addParametricCallback(0.1, () -> armMotor.setTargetPosition(1400))
                .build();

        /* This is our path6 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        chamberSpec2 = follower.pathBuilder()
                .addPath(
                        // Line 12
                        new BezierCurve(
                                new Point(10.500, 16.500, Point.CARTESIAN),
                                new Point(14.327, 70.907, Point.CARTESIAN),
                                new Point(35.000, 68.300, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                .build();

        /* This is our path6 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        wallGrab3 = follower.pathBuilder()
                .addPath(
                        // Line 13
                        new BezierCurve(
                                new Point(35.000, 68.300, Point.CARTESIAN),
                                new Point(14.813, 71.393, Point.CARTESIAN),
                                new Point(10.500, 16.500, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                // Ready to pick up specimen
                .addParametricCallback(0.1, () -> armAngleMotor.setTargetPosition(3800))
                .addParametricCallback(0.1, () -> armMotor.setTargetPosition(1400))
                .build();

        /* This is our path6 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        chamberSpec3 = follower.pathBuilder()
                .addPath(
                        // Line 14
                        new BezierCurve(
                                new Point(10.500, 16.500, Point.CARTESIAN),
                                new Point(15.541, 73.821, Point.CARTESIAN),
                                new Point(35.000, 71.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                .build();

        /* This is our path6 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        wallGrab4 = follower.pathBuilder()
                .addPath(
                        // Line 15
                        new BezierCurve(
                                new Point(35.000, 71.000, Point.CARTESIAN),
                                new Point(14.813, 72.364, Point.CARTESIAN),
                                new Point(10.500, 16.500, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                // Ready to pick up specimen
                .addParametricCallback(0.1, () -> armAngleMotor.setTargetPosition(3800))
                .addParametricCallback(0.1, () -> armMotor.setTargetPosition(1400))
                .build();

        /* This is our path6 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        chamberSpec4 = follower.pathBuilder()
                .addPath(
                        // Line 16
                        new BezierCurve(
                                new Point(10.500, 16.500, Point.CARTESIAN),
                                new Point(17.484, 77.464, Point.CARTESIAN),
                                new Point(35.000, 75.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                .build();

        /* This is our path6 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        park = follower.pathBuilder()
                .addPath(
                        // Line 17
                        new BezierCurve(
                                new Point(35.000, 75.000, Point.CARTESIAN),
                                new Point(15.298, 76.735, Point.CARTESIAN),
                                new Point(15.000, 15.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addParametricCallback(0.1, () -> armAngleMotor.setTargetPosition(3800))
                .addParametricCallback(0.1, () -> armMotor.setTargetPosition(1400))
                .build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // This is our start state

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the score1Pose's position */                 /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(initForward,true); // Follow path1

                // Set motor positions
                    armMotor.setTargetPosition(1100); // Up to enter chambers
                    armAngleMotor.setTargetPosition(3073); // Forward and down to enter chambers
                    //ascend.setTargetPosition(27500); // Up, stay until endgame for hang

                // Set motor modes, only once, after positions are set
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armAngleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //ascend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // Set motor powers, only once, after modes are set
                    armMotor.setPower(1);
                    armAngleMotor.setPower(1);
                    //ascend.setPower(1);

                    setPathState(1); // Now following path1
                break;

                case 1: // Following path1
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the behindSpike1Pose's position */
                if(!follower.isBusy() && armMotor.getCurrentPosition() > 3600) { // If path1 has reached its end
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(initScore,true); // Follow path2

                    armAngleMotor.setTargetPosition(2530); // Up to clip specimen

                    setPathState(2); // Now following path2
                }
                break;

                case 2: // Following path2
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the score1Pose's position */
                if(!follower.isBusy()) { // If path2 has reached its end
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(travelToSpike1,true); // Follow path3
                    setPathState(3); // Now following path3
                }
                break;

                case 3: // Following path3
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) { // If path3 has reached its end
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(pushSpike1,true); // Follow path4
                    setPathState(4); // Now following path4
                }
                break;

                case 4: // Following path4
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the score1Pose's position */
                if(!follower.isBusy()) { // If path4 has reached its end
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(travelToSpike2,true); // Follow path5
                    setPathState(5); // Now following path5
                }
                break;

                case 5: // Following path5
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) { // If path5 has reached its end
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(pushSpike2, true); // Follow path6
                    setPathState(6); // Now following path6
                }
                break;

                case 6: // Following path6
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the score1Pose's position */
                if(!follower.isBusy()) { // If path6 has reached its end
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(travelToSpike3,true); // Follow path7
                    setPathState(7); // Now following path7
                }
                break;

                case 7: // Following path7
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the score1Pose's position */
                if(!follower.isBusy()) { // If path7 has reached its end
                    // Use this to reset Y and heading against wall
                    follower.followPath(pushSpike3,true); // Follow path8
                    armMotor.setTargetPosition(1400); // Ready to grab wall spec
                    armAngleMotor.setTargetPosition(2200); // Clear wall so you can push last spike

                    setPathState(8); // Now following path8
                }
                break;

                case 8: // Following path8
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the score1Pose's position */
                    if(!follower.isBusy()) { // If path8 has reached its end
                        follower.followPath(clearWall,true); // Follow path9
                        wheelServo.setPower(0.15); // Hold spec down while ramming
                        setPathState(9); // Now following path9
                    }
                    break;

                    case 9: // Following path9
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the score1Pose's position */
                if(!follower.isBusy()) { // If path9 has reached its end
                    if (!wallSpecWaiting) {
                        wallSpecWaitTimer.resetTimer(); // Wait for wall spec positioning
                        wallSpecWaiting = true;
                    }
                    if (wallSpecWaitTimer.getElapsedTime() > 250 && !armAngleMotor.isBusy()) {
                        wallSpecWaiting = false;
                        follower.followPath(wallGrab1, true); // Follow path10
                        setPathState(10); // Now following path10
                    }
                }
                break;

                case 10: // Following path10
                    if(!follower.isBusy()) { // If path10 has reached its end
                        wheelServo.setPower(0); // Stop servo
                        follower.followPath(chamberSpec1, true); // Follow path11
                        armMotor.setTargetPosition(3400); // Lift up specimen, ready to reorient specimen
                        armAngleMotor.setTargetPosition(2400); // Lift up specimen, ready to reorient specimen
                        setPathState(11); // Now following path11
                    }
                    break;

                    case 11: // Following path11
                    if(!follower.isBusy()) { // If path11 has reached its end
                        follower.followPath(wallGrab2, true); // Follow path12
                        //wheelServo.setPower(0.25); // Hold spec down while rammingT
                        armAngleMotor.setTargetPosition(2400); // Pull down to score specimen
                        armMotor.setTargetPosition(1000); // Pull down to score specimen
                        setPathState(12); // Now following path12
                    }
                    break;

                    case 12: // Following path12
                    if(!follower.isBusy()) { // If path12 has reached its end
                        //wheelServo.setPower(0); // Stop servo
                        follower.followPath(chamberSpec2, true); // Follow path13
                        setPathState(13); // Now following path13
                    }
                    break;

                    case 13: // Following path13
                    if(!follower.isBusy()) { // If path13 has reached its end
                        //wheelServo.setPower(0.25); // Hold spec down while rammingT
                        follower.followPath(wallGrab3, true); // Follow path14
                        armAngleMotor.setTargetPosition(2400); // Pull down to score specimen
                        armMotor.setTargetPosition(1000); // Pull down to score specimen
                        setPathState(14); // Now following path14
                    }
                    break;

                    case 14: // Following path14
                    if(!follower.isBusy()) { // If path14 has reached its end
                        //wheelServo.setPower(0); // Stop servo
                        follower.followPath(chamberSpec3, true); // Follow path15
                        setPathState(15); // Now following path15
                    }
                    break;

                    case 15: // Following path15
                    if(!follower.isBusy()) { // If path15 has reached its end
                        //wheelServo.setPower(0.25); // Hold spec down while rammingT
                        follower.followPath(wallGrab4, true); // Follow path16
                        armAngleMotor.setTargetPosition(2400); // Pull down to score specimen
                        armMotor.setTargetPosition(1000); // Pull down to score specimen
                        setPathState(16); // Now following path16
                    }
                    break;

                    case 16: // Following path16
                    if(!follower.isBusy()) { // If path16 has reached its end
                        //wheelServo.setPower(0); // Stop servo
                        follower.followPath(chamberSpec4, true); // Follow path17
                        setPathState(17); // Now following path17
                    }
                    break;

                    case 17: // Following path17
                    if(!follower.isBusy()) { // If path17 has reached its end
                        follower.followPath(park, true); // Follow path18
                        armAngleMotor.setTargetPosition(2400); // Pull down to score specimen
                        armMotor.setTargetPosition(1000); // Pull down to score specimen
                        setPathState(18); // Now following path18
                    }
                    break;

                    case 18: // Following path18
                    if(!follower.isBusy()) { // If path18 has reached its end
                        setPathState(-1); // End of auto
                    }
                    break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        wallSpecWaitTimer = new Timer();
        opmodeTimer.resetTimer();
        wallSpecWaitTimer.resetTimer();

        ascend = hardwareMap.get(DcMotorEx.class, "ascend");
        ascend2 = hardwareMap.get(DcMotorEx.class, "ascend2");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armAngleMotor = hardwareMap.get(DcMotorEx.class, "armAngleMotor");
        wheelServo = hardwareMap.get(CRServo.class, "wheelServo");
        ascendServo3 = hardwareMap.get(CRServo.class, "ascendServo3");

        // Group all motors in an array
        DcMotorEx[] allMiscMotors = new DcMotorEx[] {
                ascend, ascend2, armMotor, armAngleMotor
        };

        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx motor : allMiscMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        wallSpecWaitTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

