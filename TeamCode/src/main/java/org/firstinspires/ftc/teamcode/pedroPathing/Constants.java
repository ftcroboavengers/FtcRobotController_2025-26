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
 * Pedro + Panels constants for RoboAvengers (FTC 11 / Pedro 2.0.3 + Panels 1.0.7)
 *
 * Coordinate frame:
 *   +X = forward
 *   +Y = right
 *   +Heading = CCW
 */
public class Constants {

    /*--------------------------------------
     *  Motion model / follower tuning
     *--------------------------------------*/
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5.5);    // kg — adjust to your robot’s real weight

    /*--------------------------------------
     *  Drivetrain (Mecanum) configuration
     *--------------------------------------*/
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)
            .rightFrontMotorName("front_right_drive")
            .rightRearMotorName("back_right_drive")
            .leftRearMotorName("back_left_drive")
            .leftFrontMotorName("front_left_drive")

            // Flip directions so +Y = right
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    /*--------------------------------------
     *  Localization (GoBilda Pinpoint)
     *--------------------------------------*/
    public static PinpointConstants localizerConstants = new PinpointConstants()
            // Pod placement relative to robot center
            .forwardPodY(-5.0)  // X-pod: 5 in behind center
            .strafePodX(2.0)    // Y-pod: 2 in right of center

            // Units for Panels / telemetry
            .distanceUnit(DistanceUnit.INCH)

            // Device name in Control-Hub config
            .hardwareMapName("pinpoint")

            // Encoder + wheel type
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)

            // Directions (verified for 4-bar right-side pod)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    /*--------------------------------------
     *  Default motion constraints
     *--------------------------------------*/
    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,   // global power scale
            60,     // max translational speed (in/s)
            1.5,    // max linear acceleration (in/s²)
            1.0     // max angular acceleration (rad/s²)
    );

    /*--------------------------------------
     *  Factory method for the Pedro Follower
     *--------------------------------------*/
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
