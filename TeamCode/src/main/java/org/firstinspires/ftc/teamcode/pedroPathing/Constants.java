package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

/**
 * Pedro Pathing Constants configuration for your robot.
 *
 * - GoBilda Pinpoint is mounted at the back of the robot, 5 inches behind center.
 * - X (forward) odometry pod is near the back.
 * - Y (strafe) odometry pod is on the right side.
 * - Robot coordinate system:
 *      +X = forward
 *      +Y = right
 *      +Heading = counterclockwise
 * - Clockwise rotation decreases heading.
 */
public class Constants {

    /*--------------------------------------
     *  PID / Motion physics configuration
     *--------------------------------------*/
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5); // Approximate mass in kg

    /*--------------------------------------
     *  Drivetrain (Mecanum) configuration
     *--------------------------------------*/
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)

            .rightFrontMotorName("front_right_drive")
            .rightRearMotorName("back_right_drive")
            .leftRearMotorName("back_left_drive")
            .leftFrontMotorName("front_left_drive")
            // ✅ Flip side directions so +Y = right
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    /*--------------------------------------
     *  Localization (GoBilda Pinpoint) configuration
     *--------------------------------------*/
    public static PinpointConstants localizerConstants = new PinpointConstants()
            // Pinpoint board is ~5 inches behind center
            .forwardPodY(-5.0)

            // Strafe (Y) pod is ~2 inches to the right of center
            .strafePodX(2.0)

            // Distance units for offsets and position reporting
            .distanceUnit(DistanceUnit.INCH)

            // Hardware name from configuration
            .hardwareMapName("pinpoint")

            // GoBilda 4-bar odometry pod setup
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)

            // Encoder direction configuration:
            //   FORWARD means pod increases counts in the same direction
            //   as positive X (for forwardPod) or positive Y (for strafePod)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    /*--------------------------------------
     *  Motion constraints
     *--------------------------------------*/
    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,   // max power scaling
            100,    // max translational speed (in/sec)
            1,      // max acceleration
            1       // max angular acceleration
    );

    /*--------------------------------------
     *  Follower factory method
     *--------------------------------------*/
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
