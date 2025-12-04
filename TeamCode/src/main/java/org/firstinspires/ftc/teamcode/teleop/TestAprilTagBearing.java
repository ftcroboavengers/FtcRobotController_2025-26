package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;
import java.util.List;
@Disabled
@TeleOp(name = "TEST: AprilTag Bearing", group = "Debug")
public class TestAprilTagBearing extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    @Override
    public void runOpMode() throws InterruptedException {

        // -------------------------
        // Build AprilTag processor
        // -------------------------
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(tagProcessor)
                .enableLiveView(true)
                .build();

        telemetry.addLine(">>> Press START to read AprilTag bearings…");
        telemetry.update();

        waitForStart();

        // -------------------------
        // Main loop
        // -------------------------
        while (opModeIsActive()) {

            AprilTagDetection tag = getAnyTag();

            if (tag != null && tag.ftcPose != null) {

                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("Range (in)", "%.2f", tag.ftcPose.range);
                telemetry.addData("Bearing (deg)", "%.2f", tag.ftcPose.bearing);
                telemetry.addData("Yaw (deg)", "%.2f", tag.ftcPose.yaw);

            } else {
                telemetry.addLine("NO TAG VISIBLE");
            }

            telemetry.update();
            sleep(40);
        }
    }

    /** Returns any visible tag (usually closest). */
    private AprilTagDetection getAnyTag() {
        List<AprilTagDetection> list = new ArrayList<>(tagProcessor.getDetections());
        if (list.isEmpty()) return null;
        return list.get(0);
    }
}
