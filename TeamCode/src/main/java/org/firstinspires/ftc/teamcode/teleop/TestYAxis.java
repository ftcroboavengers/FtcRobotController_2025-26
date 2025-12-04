package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Disabled
@TeleOp(name = "Test Y Axis", group = "RoboAvengers")
public class TestYAxis extends LinearOpMode {

    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() throws InterruptedException {

        // ---- Initialize Pinpoint ----
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

            // Set the encoder configuration for goBILDA 4-bar odometry pods
            pinpoint.setEncoderResolution(
                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
            );

            // Make sure both encoder directions are set to FORWARD unless yours is different
            pinpoint.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD
            );

            // Zero the position and IMU
            pinpoint.resetPosAndIMU();

        } catch (Exception e) {
            telemetry.addLine("ERROR: Could not initialize Pinpoint!");
            telemetry.update();
            return;
        }

        telemetry.addLine("Ready — Start, then physically push the robot RIGHT");
        telemetry.update();

        waitForStart();

        // -----------------------------------------
        // MAIN LOOP: PRINT PINPOINT Y POSITION
        // -----------------------------------------
        while (opModeIsActive()) {
            pinpoint.update();

            double yInches = pinpoint.getPosY(DistanceUnit.INCH);

            telemetry.addData("Pinpoint Y (inches)", yInches);
            telemetry.addLine("Push robot RIGHT and watch if this number goes UP or DOWN.");
            telemetry.update();
        }
    }
}
