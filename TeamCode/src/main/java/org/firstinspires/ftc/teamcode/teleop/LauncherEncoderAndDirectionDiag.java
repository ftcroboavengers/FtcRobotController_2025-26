package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Launcher Encoder & Direction Diag", group = "Diagnostics")
public class LauncherEncoderAndDirectionDiag extends LinearOpMode {

    private DcMotorEx left, right;

    private boolean velocityMode = false;   // false = power mode, true = velocity mode
    private double targetTPS = 900;         // ticks/sec target for velocity mode
    private static final double TPR = 28.0; // ticks per revolution (GoBilda)

    // Helpers to avoid repeats
    private boolean lastDpadUp = false, lastDpadDown = false, lastDpadLeft = false, lastDpadRight = false;
    private boolean lastX = false, lastY = false, lastA = false, lastB = false;
    private boolean lastLB = false, lastRB = false;

    @Override
    public void runOpMode() throws InterruptedException {
        left  = hardwareMap.get(DcMotorEx.class, "left_launcher");
        right = hardwareMap.get(DcMotorEx.class, "right_launcher");

        // Start sane: encoders reset; power mode (manual power) so you can verify spin direction first.
        resetEncoders();
        setRunWithoutEncoder();

        // Start with both directions FORWARD. You can flip each side live.
        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addLine("Launcher Encoder & Direction Diagnostic");
        telemetry.addLine("Controls:");
        telemetry.addLine("  A = BOTH FORWARD,  B = BOTH REVERSE");
        telemetry.addLine("  X = LEFT toggle dir,  Y = RIGHT toggle dir");
        telemetry.addLine("  Dpad UP = Velocity mode,  Dpad DOWN = Power mode");
        telemetry.addLine("  Dpad LEFT = Reset encoders");
        telemetry.addLine("  Dpad RIGHT = Toggle one/both selection (LB=left only, RB=right only; none=both)");
        telemetry.addLine("  Triggers: RT = forward, LT = reverse (in Power mode)");
        telemetry.addLine("  In Velocity mode: use GP2 dpad up/down to change target (+/- 200 tps)");
        telemetry.addLine("Press Play, then test spin direction in Power mode first.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- Mode switches ---
            if (pressedOnce(gamepad1.dpad_up, lastDpadUp)) {
                velocityMode = true;
                setRunUsingEncoder();
            }
            if (pressedOnce(gamepad1.dpad_down, lastDpadDown)) {
                velocityMode = false;
                setRunWithoutEncoder();
            }
            if (pressedOnce(gamepad1.dpad_left, lastDpadLeft)) {
                resetEncoders();
            }

            // --- Direction management ---
            if (pressedOnce(gamepad1.a, lastA)) {
                left.setDirection(DcMotorSimple.Direction.FORWARD);
                right.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            if (pressedOnce(gamepad1.b, lastB)) {
                left.setDirection(DcMotorSimple.Direction.REVERSE);
                right.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            if (pressedOnce(gamepad1.x, lastX)) {
                left.setDirection(left.getDirection() == DcMotorSimple.Direction.FORWARD
                        ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            }
            if (pressedOnce(gamepad1.y, lastY)) {
                right.setDirection(right.getDirection() == DcMotorSimple.Direction.FORWARD
                        ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            }

            // --- Target tweak in velocity mode (use gamepad2) ---
            if (gamepad2.dpad_up)   targetTPS += 200;
            if (gamepad2.dpad_down) targetTPS = Math.max(0, targetTPS - 200);

            // --- Which motors to drive: LB = left only, RB = right only, none = both ---
            boolean leftOnly = gamepad1.left_bumper && !gamepad1.right_bumper;
            boolean rightOnly = gamepad1.right_bumper && !gamepad1.left_bumper;
            boolean both = !leftOnly && !rightOnly;

            // --- Drive motors based on mode ---
            if (!velocityMode) {
                // Power mode: triggers give -1..+1 power
                double power = clamp(gamepad1.right_trigger - gamepad1.left_trigger, -1, 1);
                if (both || leftOnly)  left.setPower(power);
                if (both || rightOnly) right.setPower(power);
            } else {
                // Velocity mode: setVelocity to targetTPS (ticks/sec)
                if (both || leftOnly)  left.setVelocity(targetTPS);
                if (both || rightOnly) right.setVelocity(targetTPS);
            }

            // --- Telemetry ---
            telemetry.addData("Mode", velocityMode ? "Velocity (RUN_USING_ENCODER)" : "Power (RUN_WITHOUT_ENCODER)");
            telemetry.addData("Target TPS", "%.0f", targetTPS);
            telemetry.addData("Left Dir", left.getDirection());
            telemetry.addData("Right Dir", right.getDirection());
            telemetry.addData("Drive Sel", both ? "BOTH" : (leftOnly ? "LEFT only" : "RIGHT only"));
            telemetry.addData("Left Pos", left.getCurrentPosition());
            telemetry.addData("Right Pos", right.getCurrentPosition());
            telemetry.addData("Left Vel", "%.1f", left.getVelocity());
            telemetry.addData("Right Vel", "%.1f", right.getVelocity());
            telemetry.update();

            // remember buttons
            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;
            lastDpadLeft = gamepad1.dpad_left;
            lastDpadRight = gamepad1.dpad_right;
            lastA = gamepad1.a; lastB = gamepad1.b; lastX = gamepad1.x; lastY = gamepad1.y;
            lastLB = gamepad1.left_bumper; lastRB = gamepad1.right_bumper;
        }

        // stop on exit
        left.setPower(0);
        right.setPower(0);
    }

    private void resetEncoders() {
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // return to whichever mode is active
        if (velocityMode) setRunUsingEncoder(); else setRunWithoutEncoder();
    }

    private void setRunUsingEncoder() {
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setRunWithoutEncoder() {
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static boolean pressedOnce(boolean current, boolean last) {
        return current && !last;
    }
}
