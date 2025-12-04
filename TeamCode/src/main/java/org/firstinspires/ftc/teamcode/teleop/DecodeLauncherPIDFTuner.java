package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@TeleOp(name = "Launcher PIDF Tuner", group = "RoboAvengers")
public class DecodeLauncherPIDFTuner extends LinearOpMode {

    private DcMotorEx leftLauncher, rightLauncher;
    private DcMotor intake;

    // ---------------- PIDF GAINS (start values) ----------------
    private static final double P_GAIN = 25.0;
    private static final double I_GAIN = 0.0;
    private static final double D_GAIN = 5.0;
    private double F_GAIN = 12.0;  // tunable in real time

    // ---------------- RPM / Velocity ----------------
    private static final double TPR = 28.0; // ticks per revolution (adjust if using gearbox)
    private double targetRPM = 4000;
    private double targetTPS = rpmToTicksPerSec(targetRPM);

    private static double rpmToTicksPerSec(double rpm) {
        return rpm * TPR / 60.0;
    }

    // ---------------- Unjam timing ----------------
    private boolean unjamming = false;
    private ElapsedTime unjamTimer = new ElapsedTime();
    private static final double UNJAM_TIME_SEC = 1.0;

    // ---------------- Debounce ----------------
    private boolean prevUp = false;
    private boolean prevDown = false;
    private boolean prevRight = false;
    private boolean prevLeft = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // MAP MOTORS
        leftLauncher  = getMotorEx("left_launcher");
        rightLauncher = getMotorEx("right_launcher");
        intake        = getDcMotor("intake");

        // INTAKE SETUP
        if (intake != null) {
            intake.setZeroPowerBehavior(BRAKE);
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        // LAUNCHER SETUP
        if (leftLauncher != null) {
            leftLauncher.setZeroPowerBehavior(BRAKE);
            leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
            leftLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftLauncher.setVelocityPIDFCoefficients(P_GAIN, I_GAIN, D_GAIN, F_GAIN);
        }

        if (rightLauncher != null) {
            rightLauncher.setZeroPowerBehavior(BRAKE);
            rightLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
            rightLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightLauncher.setVelocityPIDFCoefficients(P_GAIN, I_GAIN, D_GAIN, F_GAIN);
        }

        telemetry.addLine("Launcher PIDF Tuning Mode Ready");
        telemetry.addLine("Use D-pad + bumpers + triggers for tuning");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ---------------- RPM TUNING ----------------
            boolean up = gamepad2.dpad_up;
            boolean down = gamepad2.dpad_down;

            if (up && !prevUp) targetRPM += 100;
            if (down && !prevDown) {
                targetRPM -= 100;
                if (targetRPM < 500) targetRPM = 500;
            }
            prevUp = up;
            prevDown = down;

            targetTPS = rpmToTicksPerSec(targetRPM);

            // ---------------- F-GAIN TUNING ----------------
            boolean right = gamepad2.dpad_right;
            boolean left = gamepad2.dpad_left;

            if (right && !prevRight) F_GAIN += 0.5;
            if (left && !prevLeft) {
                F_GAIN -= 0.5;
                if (F_GAIN < 0) F_GAIN = 0;
            }
            prevRight = right;
            prevLeft = left;

            // Apply updated PIDF gains live
            if (leftLauncher != null)
                leftLauncher.setVelocityPIDFCoefficients(P_GAIN, I_GAIN, D_GAIN, F_GAIN);

            if (rightLauncher != null)
                rightLauncher.setVelocityPIDFCoefficients(P_GAIN, I_GAIN, D_GAIN, F_GAIN);

            // ---------------- LAUNCHER SPIN CONTROL ----------------
            if (gamepad2.right_bumper && !unjamming) {
                startLaunchers();
            }

            if (gamepad2.left_bumper && !unjamming) {
                stopLaunchers();
            }

            // ---------------- INTAKE CONTROL (for feeding rings) ----------------
            if (intake != null && !unjamming) {
                double in = gamepad2.right_trigger;  // feed into launcher
                double out = gamepad2.left_trigger;  // reverse intake

                double p = 0;
                double scale = 1.0; // full power for consistent feeding

                if (in > 0.05 || out > 0.05) {
                    p = (in - out) * scale;
                }
                intake.setPower(p);

                // Optional safety: B to stop intake
                if (gamepad2.b) intake.setPower(0);
            }

            // ---------------- UNJAM (X) ----------------
            if (gamepad2.x && !unjamming) {
                startUnjam();
            }
            if (unjamming) {
                updateUnjam();
            }

            // ---------------- TELEMETRY ----------------
            double leftVel = leftLauncher != null ? leftLauncher.getVelocity() : 0;
            double rightVel = rightLauncher != null ? rightLauncher.getVelocity() : 0;

            telemetry.addLine("===== LAUNCHER PIDF TUNING =====");
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Target TPS", targetTPS);
            telemetry.addData("Left Vel (tps)", leftVel);
            telemetry.addData("Right Vel (tps)", rightVel);
            telemetry.addData("Left Error", leftVel - targetTPS);
            telemetry.addData("Right Error", rightVel - targetTPS);

            telemetry.addLine("\n----- PIDF VALUES -----");
            telemetry.addData("P", P_GAIN);
            telemetry.addData("I", I_GAIN);
            telemetry.addData("D", D_GAIN);
            telemetry.addData("F (tunable)", F_GAIN);

            telemetry.addLine("\n----- CONTROLS -----");
            telemetry.addLine("D-pad Up:     Increase RPM (+100)");
            telemetry.addLine("D-pad Down:   Decrease RPM (-100)");
            telemetry.addLine("D-pad Right:  Increase F (+0.5)");
            telemetry.addLine("D-pad Left:   Decrease F (-0.5)");
            telemetry.addLine("Right Bumper: Spin UP launchers");
            telemetry.addLine("Left Bumper:  STOP launchers");
            telemetry.addLine("Right Trigger: Intake FEED");
            telemetry.addLine("Left Trigger:  Intake REVERSE");
            telemetry.addLine("B: STOP intake");
            telemetry.addLine("X: UNJAM");

            telemetry.update();
        }

        stopLaunchers();
        if (intake != null) intake.setPower(0);
    }

    // ---------------- Helper Methods ----------------

    private void startLaunchers() {
        double adjusted = getVoltageCompensatedVelocity(targetTPS);

        if (leftLauncher != null)
            leftLauncher.setVelocity(adjusted);

        if (rightLauncher != null)
            rightLauncher.setVelocity(adjusted);
    }

    private void stopLaunchers() {
        if (leftLauncher != null) leftLauncher.setPower(0);
        if (rightLauncher != null) rightLauncher.setPower(0);
    }

    private void startUnjam() {
        unjamming = true;
        unjamTimer.reset();

        if (leftLauncher != null) leftLauncher.setPower(-0.4);
        if (rightLauncher != null) rightLauncher.setPower(-0.4);
        if (intake != null) intake.setPower(-1.0);
    }

    private void updateUnjam() {
        if (unjamTimer.seconds() > UNJAM_TIME_SEC) {
            unjamming = false;
            stopLaunchers();
            if (intake != null) intake.setPower(0);
        }
    }

    private double getVoltageCompensatedVelocity(double targetTicksPerSec) {
        double nominalVoltage = 13.0;
        double currentVoltage = 12.0;

        try {
            currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        } catch (Exception e) {}

        if (currentVoltage <= 0) currentVoltage = 12.0;

        return targetTicksPerSec * (nominalVoltage / currentVoltage);
    }

    private DcMotorEx getMotorEx(String name) {
        try {
            return hardwareMap.get(DcMotorEx.class, name);
        } catch (Exception e) {
            return null;
        }
    }

    private DcMotor getDcMotor(String name) {
        try {
            return hardwareMap.get(DcMotor.class, name);
        } catch (Exception e) {
            return null;
        }
    }
}
