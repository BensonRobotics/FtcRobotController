package pedroPathing.constants;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.DRIVE_ENCODERS;

        FollowerConstants.leftFrontMotorName = "front-left-drive";
        FollowerConstants.leftRearMotorName = "back-left-drive";
        FollowerConstants.rightFrontMotorName = "front-right-drive";
        FollowerConstants.rightRearMotorName = "back-right-drive";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 11.567; // Kilograms

        FollowerConstants.xMovement = 57.8741;
        FollowerConstants.yMovement = 52.295;

        FollowerConstants.forwardZeroPowerAcceleration = -41.278;
        FollowerConstants.lateralZeroPowerAcceleration = -59.7819;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0);
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(2,0,0.1,0);
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.1,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.1,0,0,0.6,0);
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.1,0,0,0.6,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 4;
        FollowerConstants.centripetalScaling = 0.0005;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;

        // Custom constants below
        FollowerConstants.useBrakeModeInTeleOp = true;
        FollowerConstants.useVoltageCompensationInAuto = true;
        FollowerConstants.useVoltageCompensationInTeleOp = true;
        FollowerConstants.nominalVoltage = 10;
        FollowerConstants.maxPower = 0.25; // Speed limit for minimizing wheel slippage
        // Only use until we get an odometry sensor
    }
}
