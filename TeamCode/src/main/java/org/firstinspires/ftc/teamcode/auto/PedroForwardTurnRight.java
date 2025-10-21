package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Forward 20 + Turn Right", group = "Autonomous")
@Configurable
public class PedroForwardTurnRight extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private int pathState;
    private Paths paths;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);

        // Starting pose — adjust heading if robot drives sideways
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(0)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        // Begin with forward movement
        follower.followPath(paths.forward20);
        pathState = 0;
    }

    @Override
    public void loop() {
        follower.update();

        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    pathState = 1;
                    follower.followPath(paths.turnRight);
                }
                break;

            case 1:
                if (!follower.isBusy()) {
                    pathState = 2;
                    // All motion stops automatically here
                }
                break;
        }

        telemetry.addData("State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    /** Defines paths used in this autonomous. */
    public static class Paths {
        public PathChain forward20;
        public PathChain turnRight;

        public Paths(Follower follower) {
            // Path 1: move forward 20 inches
            forward20 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(0, 0, Math.toRadians(0)),
                            new Pose(0, 20, Math.toRadians(0))
                    ))
                    .build();

            // Path 2: in-place right turn (0° → −90°)
            turnRight = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(0, 20, Math.toRadians(0)),
                            new Pose(0, 20, Math.toRadians(0))
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                    .build();
        }
    }
}
