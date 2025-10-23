package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

/**
 * Pinpoint Direction Check
 *
 * Push the robot and watch telemetry:
 *  - Push forward  → X should increase
 *  - Push right    → Y should increase
 *  - Turn clockwise → Heading should increase
 *
 * If any behave backwards, fix them with setHeadingOffset(180)
 * or change encoder directions in your PinpointConstants.
 */
@TeleOp(name = "Pinpoint Direction Check", group = "Test")
public class PinpointDirectionCheck extends LinearOpMode {

    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing Pinpoint...");
        telemetry.update();

        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        } catch (Exception e) {
            telemetry.addLine("❌ ERROR: Pinpoint not found in hardware map!");
            telemetry.update();
            sleep(4000);
            return;
        }

        // --- Basic setup ---
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,   // forward pod
                GoBildaPinpointDriver.EncoderDirection.FORWARD   // strafe pod
        );
        pinpoint.setOffsets(0, 0, DistanceUnit.MM);
        pinpoint.resetPosAndIMU();

        telemetry.addLine("✅ Pinpoint ready. Press PLAY to begin direction test.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            pinpoint.update();

            double x = pinpoint.getPosX(DistanceUnit.MM);
            double y = pinpoint.getPosY(DistanceUnit.MM);
            double heading = pinpoint.getHeading(AngleUnit.DEGREES);

            telemetry.addData("X (mm)", "%.1f", x);
            telemetry.addData("Y (mm)", "%.1f", y);
            telemetry.addData("Heading (deg)", "%.1f", heading);
            telemetry.addLine("-------------------------------");
            telemetry.addLine("Push robot:");
            telemetry.addLine("Forward → X increases");
            telemetry.addLine("Right   → Y increases");
            telemetry.addLine("Turn CW → Heading increases");
            telemetry.update();

            sleep(50);
        }
    }
}
