package pedroPathing.constants;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.constants.DriveEncoderConstants;

public class LConstants {
    static {
        DriveEncoderConstants.forwardTicksToInches = 1; // Tune
        DriveEncoderConstants.strafeTicksToInches = 1;
        DriveEncoderConstants.turnTicksToInches = 1;

        DriveEncoderConstants.robot_Width = 16.287; // Inches, centers of wheels
        DriveEncoderConstants.robot_Length = 13.228; // Inches, centers of wheels

        DriveEncoderConstants.leftFrontEncoderDirection = Encoder.REVERSE;
        DriveEncoderConstants.rightFrontEncoderDirection = Encoder.FORWARD;
        DriveEncoderConstants.leftRearEncoderDirection = Encoder.REVERSE;
        DriveEncoderConstants.rightRearEncoderDirection = Encoder.FORWARD;
    }
}




