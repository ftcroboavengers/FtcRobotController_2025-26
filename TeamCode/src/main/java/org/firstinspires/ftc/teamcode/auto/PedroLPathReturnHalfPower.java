package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro L-Path Return (Half Power)", group = "Test")
public class PedroLPathReturnHalfPower extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.5);

        Pose startPose = new Pose(0, 0, 0);
        Pose forwardPose = new Pose(1016, 0, 0);            // 40 in → mm
        Pose turnPose = new Pose(1016, 0, Math.toRadians(-90));
        Pose forwardAgainPose = new Pose(1016, -508, Math.toRadians(-90)); // 20 in → mm
        Pose returnPose = new Pose(0, 0, 0);

        Path forwardPath = new Path(new BezierLine(startPose, forwardPose));
        Path turnPath = new Path(new BezierLine(forwardPose, turnPose));
        Path forwardAgainPath = new Path(new BezierLine(turnPose, forwardAgainPose));
        Path returnPath = new Path(new BezierLine(forwardAgainPose, returnPose));

        turnPath.setLinearHeadingInterpolation(0, Math.toRadians(-90));
        returnPath.setLinearHeadingInterpolation(Math.toRadians(-90), 0);

        follower.setPose(startPose);  // Important

        telemetry.addLine("Ready for Pedro L-path test (half power)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        followPath(follower, forwardPath);
        followPath(follower, turnPath);
        followPath(follower, forwardAgainPath);
        followPath(follower, returnPath);

        telemetry.addLine("L-path and return complete!");
        telemetry.update();
    }

    private void followPath(Follower follower, Path path) {
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            Pose pose = follower.getPose();
            telemetry.addData("X (mm)", "%.1f", pose.getX());
            telemetry.addData("Y (mm)", "%.1f", pose.getY());
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.getHeading()));
            telemetry.update();
        }
    }
}
