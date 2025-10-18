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
        // 1️⃣ Create Pedro follower from constants
        Follower follower = Constants.createFollower(hardwareMap);

        // (no follower.enablePanels(); — not in your version)

        // Optionally limit max power
        follower.setMaxPower(0.5);

        // --- Define poses ---
        Pose startPose = new Pose(0, 0, 0);
        Pose forwardPose = new Pose(40, 0, 0);
        Pose turnPose = new Pose(40, 0, -90);
        Pose forwardAgainPose = new Pose(40, -20, -90);
        Pose returnPose = new Pose(0, 0, 0);

        // --- Define paths ---
        Path forwardPath = new Path(new BezierLine(startPose, forwardPose));
        Path turnPath = new Path(new BezierLine(forwardPose, turnPose));
        Path forwardAgainPath = new Path(new BezierLine(turnPose, forwardAgainPose));
        Path returnPath = new Path(new BezierLine(forwardAgainPose, returnPose));

        // --- Heading interpolation for turns ---
        turnPath.setLinearHeadingInterpolation(0, Math.toRadians(-90));
        returnPath.setLinearHeadingInterpolation(Math.toRadians(-90), 0);

        //  (Place telemetry setup here — before waitForStart)
        telemetry.addLine("Ready for Pedro L-path test (half power)");
        telemetry.update();

        // 2️⃣ Wait for Start button
        waitForStart();
        if (isStopRequested()) return;

        // 3️⃣ Run your path sequence
        followPath(follower, forwardPath);
        followPath(follower, turnPath);
        followPath(follower, forwardAgainPath);
        followPath(follower, returnPath);

        telemetry.addLine("L-path and return complete!");
        telemetry.update();
    }

    // 4️⃣ Your helper method — this is where you update telemetry continuously
    private void followPath(Follower follower, Path path) {
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();

            // 🟣 PLACE THIS TELEMETRY SNIPPET HERE
            Pose pose = follower.getPose();
            telemetry.addData("X", "%.1f", pose.getX());
            telemetry.addData("Y", "%.1f", pose.getY());
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.getHeading()));
            telemetry.update();

            // 👇 Optional: send this to external dashboard (if you use one)
            // FtcDashboard.getInstance().getTelemetry().addData("X", pose.getX());
            // FtcDashboard.getInstance().getTelemetry().addData("Y", pose.getY());
            // FtcDashboard.getInstance().getTelemetry().update();
        }
    }
}
