package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

@TeleOp(name = "Pinpoint Diagnostic Test", group = "Test")
public class PinpointTest extends LinearOpMode {

    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing Pinpoint...");
        telemetry.update();

        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        } catch (Exception e) {
            telemetry.addLine(" ERROR: Pinpoint not found in hardware map.");
            telemetry.update();
            sleep(5000);
            return;
        }

        // Basic configuration
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpoint.setOffsets(0, 0, DistanceUnit.MM);
        pinpoint.resetPosAndIMU();

        telemetry.addLine("✅ Pinpoint ready. Press PLAY to begin tracking test.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            pinpoint.update();

            telemetry.addData("Status", pinpoint.getDeviceStatus());
            telemetry.addData("Firmware", pinpoint.getDeviceVersion());
            telemetry.addData("X (mm)", pinpoint.getPosX(DistanceUnit.MM));
            telemetry.addData("Y (mm)", pinpoint.getPosY(DistanceUnit.MM));
            telemetry.addData("Heading (deg)", pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Loop Time (µs)", pinpoint.getLoopTime());
            telemetry.addData("Frequency (Hz)", "%.1f", pinpoint.getFrequency());
            telemetry.update();

            sleep(50);
        }
    }
}
