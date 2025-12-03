package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.ArrayList;

@TeleOp(name = "TEST: AprilTag Camera", group = "Tests")
public class Test_AprilTagCamera extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    @Override
    public void runOpMode() {

        telemetry.addLine("Initializing camera...");
        telemetry.update();

        // ------------------------
        // Initialize AprilTag system
        // ------------------------
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

        telemetry.addLine("Camera READY");
        telemetry.addLine("Show a tag to the camera.");
        telemetry.update();

        waitForStart();

        // ------------------------
        // MAIN LOOP
        // ------------------------
        while (opModeIsActive()) {

            // Copy detections safely (avoids list changing mid-read)
            List<AprilTagDetection> list = new ArrayList<>(tagProcessor.getDetections());
            telemetry.addData("Detections", list.size());

            if (!list.isEmpty()) {
                AprilTagDetection tag = list.get(0);

                telemetry.addData("Tag ID", tag.id);

                // Only read pose IF it's available
                if (tag.ftcPose != null) {
                    telemetry.addData("Range (m)",  "%.2f", tag.ftcPose.range);
                    telemetry.addData("Bearing (°)", "%.2f", tag.ftcPose.bearing);
                    telemetry.addData("Yaw (°)",     "%.2f", tag.ftcPose.yaw);
                    telemetry.addData("Pitch (°)",   "%.2f", tag.ftcPose.pitch);
                    telemetry.addData("Roll (°)",    "%.2f", tag.ftcPose.roll);
                } else {
                    telemetry.addLine("FTC Pose: null (tag too close or detection unstable)");
                }
            }

            telemetry.update();
        }

        visionPortal.close();
    }
}