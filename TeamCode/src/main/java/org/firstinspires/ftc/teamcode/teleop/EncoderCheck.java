package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Simple diagnostic to verify that both launcher encoders
 * are wired correctly and report position + velocity.
 *
 * Spin the flywheels by hand or run them gently with setPower().
 */
@TeleOp(name = "Encoder Check", group = "Diagnostics")
public class EncoderCheck extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx leftLauncher = hardwareMap.get(DcMotorEx.class, "left_launcher");
        DcMotorEx rightLauncher = hardwareMap.get(DcMotorEx.class, "right_launcher");

        // Ensure both are using encoders
        leftLauncher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftLauncher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightLauncher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Encoder check ready. Spin motors by hand.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Left Enc Pos", leftLauncher.getCurrentPosition());
            telemetry.addData("Right Enc Pos", rightLauncher.getCurrentPosition());
            telemetry.addData("Left Vel", leftLauncher.getVelocity());
            telemetry.addData("Right Vel", rightLauncher.getVelocity());
            telemetry.update();
        }
    }
}

