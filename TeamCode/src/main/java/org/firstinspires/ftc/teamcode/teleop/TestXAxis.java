package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Disabled
@TeleOp(name = "Test Pinpoint X-Axis", group = "Testing")
public class TestXAxis extends LinearOpMode {

    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() throws InterruptedException {

        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

            // Match your competition settings
            pinpoint.setEncoderResolution(
                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
            );

            pinpoint.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD, // X pod
                    GoBildaPinpointDriver.EncoderDirection.FORWARD  // Y pod
            );

            pinpoint.setOffsets(0, 0, DistanceUnit.MM);
            pinpoint.resetPosAndIMU();

        } catch (Exception e) {
            pinpoint = null;
        }

        telemetry.addLine("Pinpoint X Test Running");
        telemetry.addLine("Move robot FORWARD/BACKWARD by hand");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (pinpoint != null) pinpoint.update();

            double xInches = pinpoint != null ? pinpoint.getPosX(DistanceUnit.INCH) : 0.0;
            double yInches = pinpoint != null ? pinpoint.getPosY(DistanceUnit.INCH) : 0.0;

            double headingDeg = 0.0;
            if (pinpoint != null) {
                headingDeg = pinpoint.getHeading(AngleUnit.DEGREES);
            }

            telemetry.addData("X (in)", xInches);
            telemetry.addData("Y (in)", yInches);
            telemetry.addData("Heading (deg)", headingDeg);
            telemetry.update();
        }
    }
}
